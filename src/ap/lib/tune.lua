-- ap/lib/tune.lua : `ap tune` - in-flight identification of how the ship really responds, stored in af.tune and used by ctl.lua.
--   * torque authority per axis (roll gx, yaw gy, pitch gz): ratio of the measured angular acceleration to the one the inertia predicts
--   * actuator delay (s): from command to rate response; limits the attitude bandwidth (ctl.lua: wn <= 0.25 / delay)
--   * lateral authority per body direction (xp, xn, zp, zn): measured acceleration / commanded acceleration (finds weak sides)
-- Method: hover at 3 m. Small torque pulses with the attitude loop frozen (open loop, ~4 deg at most), then 1.2 m/s^2 lateral pushes in
-- the four body directions with the position loop open for 1.4 s each (the closed position loop then brings the ship back).
local F = require("fly")
local M = require("m")
local T = {}

T.PULSE_ALPHA = 0.10      -- rad/s^2 predicted by the inertia for each torque pulse
T.PULSE_LEN = 0.6         -- s
T.PUSH_ACC = 1.2          -- m/s^2
T.PUSH_LEN = 1.4          -- s (open loop; the position loop then brings the ship back)

-- least-squares slope of y(t)
local function slope(ts, ys)
  local n, st, sy, stt, sty = #ts, 0, 0, 0, 0
  for i = 1, n do st, sy, stt, sty = st + ts[i], sy + ys[i], stt + ts[i] ^ 2, sty + ts[i] * ys[i] end
  local den = n * stt - st * st
  if n < 3 or math.abs(den) < 1e-12 then return nil end
  return (n * sty - st * sy) / den
end

-- Rate series (t relative to the window start, w along the pulse axis, signed so the pulse is positive) of a torque pulse that starts at
-- tStart and lasts Tp with predicted acceleration alphaPred. Returns authority ratio and delay (s), or nil.
function T.analyzeTorque(ts, ws, tStart, Tp, alphaPred)
  -- baseline: rate before the pulse
  local b, nb = 0, 0
  local bt, bw = {}, {}
  for i = 1, #ts do if ts[i] < tStart - 0.05 then b, nb = b + ws[i], nb + 1; bt[#bt + 1], bw[#bw + 1] = ts[i], ws[i] end end
  if nb == 0 then return nil end
  b = b / nb
  -- a window whose baseline is still moving (the loop is settling from the previous pulse) cannot be read: skip it
  local bs = slope(bt, bw)
  if bs and math.abs(bs) > 0.2 * alphaPred then return nil end
  -- plateau: mean rate well after the pulse ended (the rate stays constant while the loop is frozen)
  local p, np = 0, 0
  for i = 1, #ts do if ts[i] > tStart + Tp + 0.45 and ts[i] < tStart + Tp + 0.8 then p, np = p + ws[i], np + 1 end end
  if np == 0 then return nil end
  p = p / np - b
  local plateau = alphaPred * Tp
  if plateau <= 0 or p <= 0.1 * plateau then return nil end
  -- time the rate reaches half the plateau (the ramp of a delayed pulse is centred at delay + Tp / 2)
  local tHalf
  for i = 2, #ts do
    local a0, a1 = ws[i - 1] - b, ws[i] - b
    if ts[i - 1] >= tStart and a0 < 0.5 * p and a1 >= 0.5 * p then
      tHalf = ts[i - 1] + (ts[i] - ts[i - 1]) * (0.5 * p - a0) / (a1 - a0)
      break
    end
  end
  if not tHalf then return nil end
  return p / plateau, tHalf - tStart - Tp / 2
end

-- Velocity series along the push direction; acceleration slope over the window after the actuator delay. Returns achieved / commanded.
function T.analyzeLateral(ts, vs, tStart, len, accCmd)
  local a, b = {}, {}
  for i = 1, #ts do
    if ts[i] >= tStart + 0.6 and ts[i] <= tStart + len then a[#a + 1], b[#b + 1] = ts[i], vs[i] end
  end
  local s = slope(a, b)
  if not s then return nil end
  return s / accCmd
end

local function median(t)
  table.sort(t)
  return t[math.ceil(#t / 2)]
end

-- Runs the whole procedure in flight. Returns tune (table) or nil, reason.
function T.run(hal, af, opts)
  opts = opts or {}
  local I = af.inertia
  local plan = {}
  local t = 8                                   -- settle first
  local axes = { { "x", "tx", I.x, "gx" }, { "z", "tz", I.z, "gz" }, { "y", "ty", I.y, "gy" } }
  for _, ax in ipairs(axes) do
    for _, sgn in ipairs({ 1, -1 }) do
      plan[#plan + 1] = { kind = "torque", t0 = t, len = 4.5, axis = ax[1], key = ax[2], inertia = ax[3], gkey = ax[4], sgn = sgn }
      t = t + 4.5
    end
  end
  for _, d in ipairs({ { "xp", 1, 0 }, { "xn", -1, 0 }, { "zp", 0, 1 }, { "zn", 0, -1 } }) do
    plan[#plan + 1] = { kind = "push", t0 = t, len = T.PUSH_LEN + 3.0, name = d[1], bx = d[2], bz = d[3] }
    t = t + T.PUSH_LEN + 3.0 + 0.8
  end
  local tEnd = t + 1
  for _, w in ipairs(plan) do w.ts, w.vs = {}, {} end

  local results = { torque = {}, delay = {}, lat = {} }
  local function finish(w)
    if w.kind == "torque" then
      local alpha = T.PULSE_ALPHA
      local ratio, delay = T.analyzeTorque(w.ts, w.vs, 0.3, T.PULSE_LEN, alpha)
      if ratio then
        results.torque[w.gkey] = results.torque[w.gkey] or {}
        table.insert(results.torque[w.gkey], ratio)
        results.delay[#results.delay + 1] = delay
      end
    else
      local g = T.analyzeLateral(w.ts, w.vs, 0.2, T.PUSH_LEN, T.PUSH_ACC)
      if g then results.lat[w.name] = g end
    end
  end

  local current, done = nil, {}
  local function hook(ctx)
    local d = ctx.dwell
    if d >= tEnd then return { finish = true } end
    local w
    for _, p in ipairs(plan) do if d >= p.t0 and d < p.t0 + p.len then w = p break end end
    if current and current ~= w and not done[current] then done[current] = true; finish(current) end
    current = w
    if not w then return nil end
    local lt = d - w.t0
    if w.kind == "torque" then
      -- body rate along the pulse axis (the controller's torque axes: tx about body X, tz about body Z, ty about body Y)
      local wv = ctx.wFd[w.axis] * w.sgn
      w.ts[#w.ts + 1], w.vs[#w.vs + 1] = lt, wv
      local ov = { freeze = (lt >= 0.2 and lt < 0.3 + T.PULSE_LEN + 0.8) }
      if lt >= 0.3 and lt < 0.3 + T.PULSE_LEN then ov.probe = { [w.key] = w.sgn * w.inertia * T.PULSE_ALPHA } end
      return ov
    else
      if not w.dir then
        -- world direction of the body axis, fixed at the start of the push
        local v = M.qrot(ctx.st.q, { x = w.bx, y = 0, z = w.bz })
        local n = math.sqrt(v.x ^ 2 + v.z ^ 2)
        w.dir = { x = v.x / n, z = v.z / n }
      end
      local vp = ctx.st.vx * w.dir.x + ctx.st.vz * w.dir.z
      w.ts[#w.ts + 1], w.vs[#w.vs + 1] = lt, vp
      -- open-loop push, then the position loop (closed again) brings the ship back to the hover point
      if lt < 0.2 + T.PUSH_LEN then
        local a = (lt >= 0.2) and T.PUSH_ACC or 0
        return { openLoop = true, ax = a * w.dir.x, az = a * w.dir.z }
      end
      return nil
    end
  end

  local s = hal.sense()
  local o = {}
  for k, v in pairs(opts) do o[k] = v end
  o.hook = hook
  -- a gentle controller for the identification flight: the ship's true delay and authority are not known yet
  local Al = require("alloc")
  local al = Al.new(af.rotors)
  if al and (al.rollTilt or al.pitchTilt) then
    -- a ship that can only translate by leaning (tandem, coaxial) needs a quick attitude loop under its position loop, and the pulses move it further
    o.ctl = o.ctl or { wnAtt = 1.0, wnYaw = 0.7, wnAlt = 0.9 }
    o.maxDrift = o.maxDrift or 30
  else
    o.ctl = o.ctl or { wnAtt = 0.7, wnYaw = 0.5, wnAlt = 0.8 }
  end
  o.maxTilt = o.maxTilt or 25
  o.slack = o.slack or 80
  local res = F.route(hal, af, { { x = s.p.x, z = s.p.z, y = s.p.y + (opts.height or 3), dwell = tEnd + 5 } }, o)
  if current and not done[current] then finish(current) end
  T.last = plan                                   -- raw series, for diagnostics
  if not res.ok then return nil, res.reason, res end
  local tune = { delay = nil, lat = {} }
  for _, k in ipairs({ "gx", "gy", "gz" }) do
    local r = results.torque[k]
    if not r or #r < 1 then return nil, "no usable response on " .. k, res end
    local sum = 0
    for _, v in ipairs(r) do sum = sum + v end
    tune[k] = sum / #r
  end
  if #results.delay < 3 then return nil, "could not measure the actuator delay", res end
  tune.delay = median(results.delay)
  for _, k in ipairs({ "xp", "xn", "zp", "zn" }) do
    if not results.lat[k] then return nil, "no usable lateral response " .. k, res end
    tune.lat[k] = results.lat[k]
  end
  return tune, nil, res
end

return T
