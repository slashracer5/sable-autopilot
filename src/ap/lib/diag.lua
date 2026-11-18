-- ap/lib/diag.lua : in-flight diagnostics (ports of the quadcopter's test flights): pushmap, gust, imbal, leanstep, yawcal.
-- Every test hovers at a few metres, runs a timed script through the flight hook (fly.lua) and returns a result table plus CSV rows.
-- All of them run inside ComputerCraft; nothing needs an external program.
local F = require("fly")
local M = require("m")
local D = {}

local function runScript(hal, af, windows, total, opts, extra)
  local cur, doneW = nil, {}
  local rows = { "t,window,x,z,y,vx,vz,tilt,wx,wy,wz" }
  local function close(w) if w and not doneW[w] then doneW[w] = true; if w.finish then w.finish(w) end end end
  local function hook(ctx)
    local d = ctx.dwell
    if d >= total then return { finish = true } end
    local w
    for _, p in ipairs(windows) do if d >= p.t0 and d < p.t0 + p.len then w = p break end end
    if cur and cur ~= w then close(cur) end
    cur = w
    local at = M.attitude(ctx.st.q)
    rows[#rows + 1] = string.format("%.2f,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.4f,%.4f,%.4f", d, w and w.name or "-", ctx.st.px, ctx.st.pz, ctx.st.y, ctx.st.vx, ctx.st.vz,
      at.tilt, ctx.wFd.x, ctx.wFd.y, ctx.wFd.z)
    if not w then return nil end
    return w.step(w, d - w.t0, ctx)
  end
  local s = hal.sense()
  local o = {}
  for k, v in pairs(opts or {}) do o[k] = v end
  o.hook = hook
  o.maxTilt = o.maxTilt or 25
  o.slack = o.slack or 120
  o.maxDrift = o.maxDrift or 25                      -- open-loop pushes legitimately take the ship several metres away
  o.ctl = o.ctl or { wnAtt = 0.9, wnAlt = 0.9 }
  for k, v in pairs(extra or {}) do o[k] = v end
  local res = F.route(hal, af, { { x = s.p.x, z = s.p.z, y = s.p.y + (opts and opts.height or 4), dwell = total + 5 } }, o)
  close(cur)
  return res, rows
end

local function saveRows(rows, name)
  local ok, err = require("airframe").saveLog("/ap_data/diag_" .. name .. ".csv", rows)
  if not ok then print("diag log not saved: " .. tostring(err)) end
end

-- body-frame direction (angle phi from body +x toward body +z) -> world unit vector (x, z), for the ship's heading right now
local function worldDir(q, phi)
  local v = M.qrot(q, { x = math.cos(phi), y = 0, z = math.sin(phi) })
  local n = math.sqrt(v.x ^ 2 + v.z ^ 2)
  return v.x / n, v.z / n
end

-- ---------------------------------------------------------------------------------------------------------------------
-- pushmap: open-loop pushes of 1.2 m/s^2 for 2 s in 8 body directions. The achieved acceleration along / across the push shows weak sides.
-- ---------------------------------------------------------------------------------------------------------------------
function D.pushmap(hal, af, opts)
  local ACC, LEN, FROM = 1.2, 2.0, 0.9
  local alloc = require("alloc").new(af.rotors)
  if alloc and (alloc.rollTilt or alloc.pitchTilt) then ACC, LEN, FROM = 0.6, 4.5, 2.4 end   -- a lean-only axis needs time to build up its lean
  local windows, results = {}, {}
  for k = 0, 7 do
    local phi = k * math.pi / 4
    local w = { name = "push" .. k * 45, t0 = 8 + (LEN + 4.5) * k, len = LEN + 4.0, phi = phi, ts = {}, vs = {}, vp = {} }
    w.step = function(w, lt, ctx)
      if not w.dir then
        local ux, uz = worldDir(ctx.st.q, w.phi)
        w.dir = { x = ux, z = uz }
      end
      local along = ctx.st.vx * w.dir.x + ctx.st.vz * w.dir.z
      local perp = -ctx.st.vx * w.dir.z + ctx.st.vz * w.dir.x
      w.ts[#w.ts + 1], w.vs[#w.vs + 1], w.vp[#w.vp + 1] = lt, along, perp
      if lt >= 0.3 and lt < 0.3 + LEN then return { openLoop = true, ax = ACC * w.dir.x, az = ACC * w.dir.z } end
      return nil
    end
    w.finish = function(w)
      -- acceleration over the part of the push after the actuator delay
      local a, b, c = {}, {}, {}
      for i = 1, #w.ts do if w.ts[i] >= FROM and w.ts[i] <= 0.3 + LEN then a[#a + 1], b[#b + 1], c[#c + 1] = w.ts[i], w.vs[i], w.vp[i] end end
      local function slope(t, y)
        local n, st, sy, stt, sty = #t, 0, 0, 0, 0
        for i = 1, n do st, sy, stt, sty = st + t[i], sy + y[i], stt + t[i] ^ 2, sty + t[i] * y[i] end
        local den = n * stt - st * st
        if n < 3 or math.abs(den) < 1e-9 then return nil end
        return (n * sty - st * sy) / den
      end
      local g, p = slope(a, b), slope(a, c)
      results[#results + 1] = { deg = math.floor(w.phi * 180 / math.pi + 0.5), gain = g and g / ACC or nil, perp = p and p / ACC or nil }
    end
    windows[#windows + 1] = w
  end
  local total = 8 + (LEN + 4.5) * 8 + 2
  local res, rows = runScript(hal, af, windows, total, opts)
  saveRows(rows, "pushmap")
  table.sort(results, function(a, b) return a.deg < b.deg end)
  local weak = {}
  for _, r in ipairs(results) do if r.gain and r.gain < 0.8 then weak[#weak + 1] = r.deg end end
  return { flight = res, table = results, weak = weak }, rows
end

-- ---------------------------------------------------------------------------------------------------------------------
-- gust: unmodelled disturbances the controller does not know about: lateral acceleration, a yaw torque kick, a collective dip.
-- ---------------------------------------------------------------------------------------------------------------------
function D.gust(hal, af, opts)
  local I = af.inertia
  local windows, out = {}, {}
  local function mk(name, t0, len, fn, summarize)
    local w = { name = name, t0 = t0, len = len, maxDev = 0, maxTilt = 0, maxYaw = 0, maxDy = 0, t = {}, dev = {} }
    w.step = function(w, lt, ctx)
      local dx, dz = ctx.st.px - ctx.ref.px, ctx.st.pz - ctx.ref.pz
      local dev = math.sqrt(dx * dx + dz * dz)
      w.maxDev = math.max(w.maxDev, dev)
      w.maxDy = math.max(w.maxDy, math.abs(ctx.st.y - ctx.ref.y))
      w.maxTilt = math.max(w.maxTilt, M.attitude(ctx.st.q).tilt)
      w.maxYaw = math.max(w.maxYaw, math.abs(M.wrap180(ctx.st.psi - ctx.ref.psi)))
      w.t[#w.t + 1], w.dev[#w.dev + 1] = lt, { dev, math.abs(ctx.st.y - ctx.ref.y), math.abs(M.wrap180(ctx.st.psi - ctx.ref.psi)) }
      return fn(lt, ctx)
    end
    w.finish = function(w) out[#out + 1] = summarize(w) end
    windows[#windows + 1] = w
  end
  local function settle(w, idx, tol, from)
    -- first time (after `from`) the deviation stays below tol until the window ends
    local last = nil
    for i = #w.t, 1, -1 do if w.dev[i][idx] > tol then last = w.t[i] break end end
    return last and math.max(0, last - from) or 0
  end
  mk("lateral 0.6 m/s2 for 2 s", 8, 10, function(lt) if lt >= 0.5 and lt < 2.5 then return { probe = { ax = 0.6, az = 0 } } end end,
    function(w) return { name = w.name, maxDev = w.maxDev, maxTilt = w.maxTilt, settle = settle(w, 1, 0.15, 2.5) } end)
  mk("yaw kick (0.15 rad/s2 for 0.5 s)", 20, 10, function(lt) if lt >= 0.5 and lt < 1.0 then return { probe = { ty = I.y * 0.15 } } end end,
    function(w) return { name = w.name, maxYaw = w.maxYaw, maxTilt = w.maxTilt, settle = settle(w, 3, 1.0, 1.0) } end)
  mk("collective -0.8 m/s2 for 1.5 s", 32, 10, function(lt) if lt >= 0.5 and lt < 2.0 then return { probe = { ay = -0.8 } } end end,
    function(w) return { name = w.name, maxDy = w.maxDy, maxTilt = w.maxTilt, settle = settle(w, 2, 0.15, 2.0) } end)
  mk("lateral -0.6 m/s2 along z for 2 s", 44, 10, function(lt) if lt >= 0.5 and lt < 2.5 then return { probe = { ax = 0, az = -0.6 } } end end,
    function(w) return { name = w.name, maxDev = w.maxDev, maxTilt = w.maxTilt, settle = settle(w, 1, 0.15, 2.5) } end)
  local res, rows = runScript(hal, af, windows, 56, opts)
  saveRows(rows, "gust")
  return { flight = res, tests = out }, rows
end

-- ---------------------------------------------------------------------------------------------------------------------
-- imbal: hidden motor imbalances (the aircraft is unevenly loaded): rotor a +pct at t1, rotor b -pct at t2, every rotor -pct at t3, all removed at t4.
-- The integrators have to find them; the learned trim (integrators) is reported at the end of each stage.
-- ---------------------------------------------------------------------------------------------------------------------
function D.imbal(hal, af, opts)
  opts = opts or {}
  local n = #af.rotors
  local pa, pb, pc = opts.pctA or 12, opts.pctB or -10, opts.pctC or -8
  local t1, t2, t3, t4, total = 10, 40, 70, 100, 128
  local state = { t = 0 }
  local function hook(rpm, t, phase)
    local d = state.t
    if d < t1 or d >= t4 then return rpm end
    local f = {}
    for j = 1, n do f[j] = 1 end
    f[1] = f[1] * (1 + pa / 100)
    if d >= t2 then f[n] = f[n] * (1 + pb / 100) end
    if d >= t3 then for j = 1, n do f[j] = f[j] * (1 + pc / 100) end end
    local out = {}
    for j = 1, n do out[j] = rpm[j] * f[j] end
    return out
  end
  local snaps, peak = {}, { tilt = 0, dev = 0, dy = 0 }
  local win = { name = "imbal", t0 = 0, len = total }
  win.step = function(w, lt, ctx)
    state.t = lt
    local at = M.attitude(ctx.st.q)
    local dx, dz = ctx.st.px - ctx.ref.px, ctx.st.pz - ctx.ref.pz
    peak.tilt = math.max(peak.tilt, at.tilt)
    peak.dev = math.max(peak.dev, math.sqrt(dx * dx + dz * dz))
    peak.dy = math.max(peak.dy, math.abs(ctx.st.y - ctx.ref.y))
    for _, ts in ipairs({ t2 - 0.5, t3 - 0.5, t4 - 0.5 }) do
      if lt >= ts and not snaps[ts] then
        local tr = ctx.ctl and { ix = ctx.ctl.ix, iz = ctx.ctl.iz, ia = ctx.ctl.ia, iy = ctx.ctl.iy } or {}
        snaps[ts] = tr
      end
    end
    return nil
  end
  local res, rows = runScript(hal, af, { win }, total, opts, { rpmHook = hook, trim = false })
  saveRows(rows, "imbal")
  return { flight = res, injected = { rotor1 = pa, rotorN = pb, all = pc }, learned = snaps, times = { t2 - 0.5, t3 - 0.5, t4 - 0.5 }, peak = peak }, rows
end

-- ---------------------------------------------------------------------------------------------------------------------
-- leanstep: whole-ship lean setpoint doublets with the position loop off (bearings level): how much acceleration does a lean angle give?
-- ---------------------------------------------------------------------------------------------------------------------
function D.leanstep(hal, af, opts)
  local g = af.g
  local windows, results = {}, {}
  local k = 0
  for _, amp in ipairs({ 0.8, 1.2, 1.6 }) do
    for _, sgn in ipairs({ 1, -1 }) do
      local w = { name = string.format("lean %+.1f", sgn * amp), t0 = 8 + 9 * k, len = 8.5, amp = amp * sgn, ts = {}, vs = {}, tilt = {} }
      k = k + 1
      w.step = function(w, lt, ctx)
        if not w.dir then
          local v = M.qrot(ctx.st.q, { x = 1, y = 0, z = 0 })
          local n = math.sqrt(v.x ^ 2 + v.z ^ 2)
          w.dir = { x = v.x / n, z = v.z / n }
        end
        local along = ctx.st.vx * w.dir.x + ctx.st.vz * w.dir.z
        local at = M.attitude(ctx.st.q)
        w.ts[#w.ts + 1], w.vs[#w.vs + 1] = lt, along
        w.tilt[#w.tilt + 1] = at.tilt
        if lt >= 0.3 and lt < 2.8 then return { openLoop = true, ax = 0, az = 0, leanTest = { w.amp * w.dir.x, w.amp * w.dir.z } } end
        return nil
      end
      w.finish = function(w)
        local a0, a1, t0, t1 = nil, nil, nil, nil
        local peakTilt, sumTilt, nTilt = 0, 0, 0
        for i = 1, #w.ts do
          if w.ts[i] >= 1.8 and not a0 then a0, t0 = w.vs[i], w.ts[i] end        -- after the lean has built up
          if w.ts[i] <= 2.8 then a1, t1 = w.vs[i], w.ts[i] end
          if w.ts[i] >= 1.8 and w.ts[i] <= 2.8 then sumTilt, nTilt = sumTilt + w.tilt[i], nTilt + 1 end
          peakTilt = math.max(peakTilt, w.tilt[i])
        end
        local acc = (a0 and a1 and t1 > t0) and (a1 - a0) / (t1 - t0) or nil
        local meanTilt = nTilt > 0 and sumTilt / nTilt or 0
        -- effect: acceleration per unit of the lean the ship ACTUALLY has (1.0 = the thrust vector gives g tan(tilt) with no lift lost);
        -- follow: how much of the commanded lean the attitude loop had reached
        local ideal = g * math.tan(meanTilt / 57.2958)
        results[#results + 1] = {
          cmd = w.amp, achieved = acc, tilt = peakTilt,
          effect = (acc and ideal > 0.05) and math.abs(acc) / ideal or nil,
          follow = math.abs(w.amp) > 0 and ideal / math.abs(w.amp) or nil,
        }
      end
      windows[#windows + 1] = w
    end
  end
  local total = 8 + 9 * k + 2
  local res, rows = runScript(hal, af, windows, total, opts)
  saveRows(rows, "leanstep")
  return { flight = res, steps = results }, rows
end

-- ---------------------------------------------------------------------------------------------------------------------
-- yawcal: tilt ONE rotor tangentially (0.1 rad for 1 s, yaw loop frozen) and measure the yaw rate it makes, per rotor.
-- ---------------------------------------------------------------------------------------------------------------------
function D.yawcal(hal, af, opts)
  local windows, results = {}, {}
  local TILT, LEN = 0.10, 1.0
  for j, r in ipairs(af.rotors) do
    local rad = math.sqrt(r.x ^ 2 + r.z ^ 2)
    local w = { name = "rotor " .. (r.zone or j), t0 = 8 + 6 * (j - 1), len = 5.5, ts = {}, wy = {}, rotor = j }
    -- tangent giving +yaw: tau_y = z Fx - x Fz  ->  tilt (tx, tz) proportional to (z, -x)
    local tx, tz = TILT * r.z / math.max(rad, 1e-6), -TILT * r.x / math.max(rad, 1e-6)
    w.step = function(w, lt, ctx)
      w.ts[#w.ts + 1], w.wy[#w.wy + 1] = lt, ctx.wFd.y
      local ov = { freeze = (lt >= 0.2 and lt < 0.3 + LEN + 1.0) }
      if lt >= 0.3 and lt < 0.3 + LEN then ov.probe = { tilt = { [j] = { tx, tz } } } end
      return ov
    end
    w.finish = function(w)
      local base, nb, plat, np = 0, 0, 0, 0
      for i = 1, #w.ts do
        if w.ts[i] < 0.25 then base, nb = base + w.wy[i], nb + 1 end
        if w.ts[i] > 0.3 + LEN + 0.45 and w.ts[i] < 0.3 + LEN + 0.9 then plat, np = plat + w.wy[i], np + 1 end
      end
      if nb > 0 and np > 0 then
        local dw = plat / np - base / nb
        -- prediction: torque = T_j * |r| * TILT (lever arm of the tangential force) over the inertia, for LEN seconds (no delay in the plateau)
        local T = af.mass * af.g / #af.rotors
        local pred = T * rad * TILT / af.inertia.y * LEN
        results[#results + 1] = { rotor = r.zone or j, dYaw = dw * 57.2958, ratio = pred > 0 and dw / pred or nil }
      end
    end
    windows[#windows + 1] = w
  end
  local total = 8 + 6 * #af.rotors + 2
  local res, rows = runScript(hal, af, windows, total, opts)
  saveRows(rows, "yawcal")
  return { flight = res, rotors = results }, rows
end

return D
