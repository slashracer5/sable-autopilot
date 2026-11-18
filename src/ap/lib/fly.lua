-- ap/lib/fly.lua : flight programs on top of the HAL + generic controller. Works on the real ship and in the simulator.
--   F.route(hal, af, points, opts): spin up on the ground, climb vertically to the first waypoint's altitude, fly the legs
--     (reference generator in nav.lua, position loop in ctl.lua), land under control at the last waypoint.
--   F.hover(hal, af, opts): the same with one waypoint above the start (opts.height, opts.secs).
-- Safety: tilt beyond opts.maxTilt cuts the motors; drift from the reference, altitude excess, a stuck climb and the time limit land the
-- ship (landing timeout cuts). The control uses the angular rate derived from the quaternion (exact); the API's angular velocity is
-- only compared with it (four sign/frame hypotheses) and reported as res.gyro.
local M = require("m")
local A = require("airframe")
local Al = require("alloc")
local Ctl = require("ctl")
local Nav = require("nav")
local F = {}
F.LOCK = "/ap_data/flight.lock"

local function norm(v)
  local n = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
  return { x = v.x / n, y = v.y / n, z = v.z / n }
end

-- heading policies of this ship: noseOn (front toward the travel direction), faceOn (weak side off the travel axis)
function F.headingModes(af, opts)
  opts = opts or {}
  local lat = af.tune and af.tune.lat
  local noseOn = A.front(af).nose and opts.faceTravel ~= false
  local faceOn = false
  if lat and opts.faceTravel ~= false then
    local lo, hi = 9, 0
    for _, k in ipairs({ "xp", "xn", "zp", "zn" }) do local v = lat[k] if v then lo, hi = math.min(lo, v), math.max(hi, v) end end
    faceOn = (opts.faceTravel == true) or (hi > 0 and lo / hi < 0.85)
  end
  return noseOn, faceOn
end

-- Heading for a leg along (ux, uz) from the present heading psiCur. Returns tgt (a heading to turn to, nil = keep), why, cap (lateral authority
-- gain along the travel direction at the heading that will be flown, or nil when the ship has not been tuned).
-- A nose-first ship keeps the front toward the travel unless the measured authority says that heading is < 70 % as good as the best one.
function F.pickHeading(af, opts, ux, uz, psiCur)
  local noseOn, faceOn = F.headingModes(af, opts)
  local lat = af.tune and af.tune.lat
  local tgt, why
  if noseOn then
    local tn = Nav.noseHeading(A.front(af), ux, uz)
    tgt, why = tn, "front toward the travel direction"
    if lat then
      local alt = Nav.bestHeading(lat, ux, uz, tn)
      if alt and Nav.headingCap(lat, ux, uz, alt) > Nav.headingCap(lat, ux, uz, tn) / 0.7 then
        tgt, why = alt, "the front direction is weak: weak side off the travel axis instead"
      end
    end
  elseif faceOn then
    tgt, why = Nav.bestHeading(lat, ux, uz, psiCur), "weak side off the travel axis"
  end
  local flown = psiCur
  if tgt and math.abs(M.wrap180(tgt - psiCur)) >= (noseOn and 12 or 0) then flown = tgt else tgt = nil end
  return tgt, why, lat and Nav.headingCap(lat, ux, uz, flown) or nil
end

-- points: list of {x=,z=,y=,speed=,dwell=} (absolute world coordinates). opts.home = true appends the start point.
function F.route(hal, af, points, opts)
  opts = opts or {}
  local say = opts.say or function() end
  local maxTilt, maxDrift = opts.maxTilt or 40, opts.maxDrift or ((opts.nav and opts.nav.plan) and ((opts.nav.maxLag or 3) + 15) or 10)
  local n = #af.rotors
  local cap = af.motorCap or 250
  local cover = { g = af.g }
  for k, v in pairs(opts.ctl or {}) do cover[k] = v end
  for k, v in pairs(opts.ctlOver or {}) do cover[k] = v end      -- `key=value` words on the command line
  local ctl, err = Ctl.new(af, cover)
  if not ctl then return { ok = false, reason = err } end
  local W = af.mass * af.g
  local T0 = Al.thrust(ctl.al, W, 0, 0)
  local hoverRpm = {}
  for j = 1, n do hoverRpm[j] = A.rpmFor(af, j, T0[j]) end

  for j, r in ipairs(af.rotors) do if r.level then hal.setLevel(j, r.level) end end
  local s0 = hal.sense()
  local ground = opts.ground or s0.p.y                        -- opts.ground: recovery after a restart (the ship is already in the air)
  local home = { x = s0.p.x, y = s0.p.y, z = s0.p.z }
  local pts = {}
  for i, p in ipairs(points) do pts[i] = p end
  if opts.home then pts[#pts + 1] = { x = home.x, z = home.z, y = pts[#pts].y, dwell = 1 } end
  local navCfg = {}
  for k, v in pairs(opts.nav or {}) do navCfg[k] = v end
  for k, v in pairs(opts.navOver or {}) do navCfg[k] = v end
  local route, rerr = Nav.build(pts, home, navCfg)
  if not route then return { ok = false, reason = rerr } end
  local R = Nav.new(route, home, navCfg)
  local res
  -- which heading a leg is flown at: nose-first ships point the FRONT along the travel direction, other ships with a weak side keep it off the
  -- travel axis (v1 faceTravel, generalised from the measured lateral authority)
  local psiTarget = M.attitude(s0.q).psi
  local lat = af.tune and af.tune.lat
  local psiNowRef = psiTarget
  local noseOn, faceOn = F.headingModes(af, opts)
  -- a ship that can only translate along one body axis by leaning (tandem: sideways) follows the reference there with a long lag: the plain-route reference
  -- accelerates gently along that axis (a 6 m sideways step overshot by 60 % with the normal 1.5 m/s^2)
  local leanOnlyAxis = (not navCfg.plan) and (ctl.al.rollTilt and "z" or (ctl.al.pitchTilt and "x")) or nil
  local baseAccel = R.cfg.accel
  if faceOn or noseOn or leanOnlyAxis then
    R.onLegStart = function(Rr, wp, ux, uz, L)
      if L >= 15 and (faceOn or noseOn) then
        local tgt, why = F.pickHeading(af, opts, ux, uz, psiTarget)
        if tgt then psiTarget = tgt; Rr.hold = true; res.turns = (res.turns or 0) + 1; say(string.format("turning to heading %.0f deg (%s)", tgt, why)) end
      end
      if leanOnlyAxis then
        local r = psiTarget / M.DEG
        local bx, bz = ux * math.cos(r) - uz * math.sin(r), ux * math.sin(r) + uz * math.cos(r)
        local frac = math.abs(leanOnlyAxis == "z" and bz or bx)
        Rr.cfg.accel = baseAccel * (1 - frac) + math.min(baseAccel, 0.4) * frac
      end
    end
  end
  if opts.airborne then R.y = s0.p.y end
  local nv = R.cfg
  local floorY = opts.floor and (ground + opts.floor) or nil           -- safety floor (transit): never fly below it until the final approach
  local lean = { locked = false, armed = false, maxFrac = 0 }
  res = { ok = true, maxTilt = 0, maxDrift = 0, maxLag = 0, rows = 0, holdErr = 0, holdN = 0, waypoints = R.log }
  -- selftests must never read or write the real /ap_data/trim.json (F.testMode), unless a test path is given
  local trimOn = (opts.trim ~= false) and (opts.trimPath ~= nil or not F.testMode)
  if trimOn then
    local tr = A.loadTrim(af, opts.trimPath)
    if tr then Ctl.setTrim(ctl, tr) res.trimLoaded = true end
  end
  local topY = ground
  for _, w in ipairs(route) do topY = math.max(topY, w.y) end
  local psi0 = M.attitude(s0.q).psi
  local up = { x = 0, y = 1, z = 0 }
  local function targetsUp() local t = {} for j = 1, n do t[j] = up end return t end
  local function cut(reason)
    res.ok = false
    res.reason = reason
    hal.stop()
  end

  local phase, tPhase = "ramp", hal.now()
  if opts.airborne then phase = "route" end
  -- flight lock: if the computer restarts mid-flight (chunk unload, crash) `ap recover` on boot finds this and lands the ship
  local lockPath = opts.lockPath or F.LOCK
  if not F.testMode or opts.lockPath then
    local lf = fs.open(lockPath, "w")
    if lf then lf.write(textutils.serialiseJSON({ ground = ground, started = os.epoch and os.epoch("utc") or 0, orig = hal.orig })) lf.close() end
  end
  local t0 = tPhase
  local last = t0
  local sample = hal.sense()
  local ref = { y = opts.airborne and s0.p.y or ground, vy = 0, psi = psi0, px = home.x, pz = home.z, vx = 0, vz = 0, ax = 0, az = 0 }
  local landRef = nil
  local stoppedT = 0
  local prevQ, wFilt, gyro = nil, { x = 0, y = 0, z = 0 }, { dot = {}, den = {} }
  local ramp = opts.ramp or 3
  local landRate = 0.6
  local routeStart, trimSnap
  local P0, pScale = nil, 1
  if opts.airborne then routeStart = hal.now() end
  local expect = Nav.estimate(route, home, R.cfg) + (topY - ground) / R.cfg.climbRate

  local levelMode = false
  local coast = { on = false, done = false }      -- drag measurement: a few seconds of level coasting at speed (nav coastSpeed / coastSecs)
  local stuckT2 = 0
  local guard = 0
  while true do
    guard = guard + 1
    if guard > 40000 then cut("loop limit") break end
    local now = hal.now()
    local dt = M.clamp(now - last, 0.01, 0.25)
    last = now
    local q = M.qnorm(sample.q)
    local at = M.attitude(q)
    local wFd = { x = 0, y = 0, z = 0 }
    if prevQ then
      local d = M.qdelta(prevQ, q)
      wFd = { x = d.x / dt, y = d.y / dt, z = d.z / dt }
    end
    prevQ = q
    wFilt.x, wFilt.y, wFilt.z = wFilt.x + 0.7 * (wFd.x - wFilt.x), wFilt.y + 0.7 * (wFd.y - wFilt.y), wFilt.z + 0.7 * (wFd.z - wFilt.z)
    local wB = { x = wFilt.x, y = wFilt.y, z = wFilt.z }
    local upB = M.qrot(M.qconj(q), up)
    local agl = sample.p.y - ground
    local fromRef = math.sqrt((sample.p.x - ref.px) ^ 2 + (sample.p.z - ref.pz) ^ 2)
    res.maxTilt = math.max(res.maxTilt, at.tilt)
    res.maxDrift = math.max(res.maxDrift, fromRef)
    local apiW = sample.w
    local apiB = M.qrot(M.qconj(q), apiW)
    local mag = math.sqrt(wFd.x ^ 2 + wFd.y ^ 2 + wFd.z ^ 2)
    if mag > 0.01 then
      local hyp = { world = apiB, body = apiW, nworld = { x = -apiB.x, y = -apiB.y, z = -apiB.z }, nbody = { x = -apiW.x, y = -apiW.y, z = -apiW.z } }
      for k, v in pairs(hyp) do
        local vm = math.sqrt(v.x ^ 2 + v.y ^ 2 + v.z ^ 2)
        gyro.dot[k] = (gyro.dot[k] or 0) + (v.x * wFd.x + v.y * wFd.y + v.z * wFd.z)
        gyro.den[k] = (gyro.den[k] or 0) + vm * mag
      end
    end

    -- sanity
    if sample.p.y ~= sample.p.y or at.tilt ~= at.tilt then cut("sensor returned NaN") break end
    -- a ship far over its tilt limit is not left to fall: first the position / lean demands are dropped and only the attitude and altitude loops
    -- level it ("level mode", until it is back under 40 % of the limit); the motors are cut only when it is nearly upside down or already close to the ground
    if at.tilt > 60 or (at.tilt > maxTilt and agl < 2) then cut(string.format("tilt %.0f deg", at.tilt)) break end
    if at.tilt > 0.75 * maxTilt and agl > 2 then
      if not levelMode then res.levelModes = (res.levelModes or 0) + 1; say(string.format("tilt %.0f deg: levelling the ship (position and lean demands dropped)", at.tilt)) end
      levelMode = true
    elseif at.tilt < 0.4 * maxTilt then levelMode = false end

    local st = {
      y = sample.p.y, vy = sample.v.y, ux = upB.x, uz = upB.z, w = wB, psi = at.psi, wy = wB.y, cosTilt = at.cosTilt, q = q,
      vx = sample.v.x, vz = sample.v.z, px = sample.p.x, pz = sample.p.z,
    }
    local cref = ref
    local rpm, targets
    if phase == "ramp" then
      local f = math.min(1, (now - tPhase) / ramp)
      rpm = {}
      for j = 1, n do rpm[j] = math.min(cap, hoverRpm[j] * 0.6 * f) end
      targets = targetsUp()
      if f >= 1 then phase, tPhase, routeStart = "route", now, now end
    else
      if phase == "route" then
        local ship = { x = sample.p.x, y = sample.p.y, z = sample.p.z, vx = sample.v.x, vz = sample.v.z, vy = sample.v.y }
        Nav.update(R, dt, ship, now)
        ref.y, ref.vy = R.y, R.vy
        ref.px, ref.pz, ref.vx, ref.vz, ref.ax, ref.az = R.x, R.z, R.vx, R.vz, R.ax, R.az
        res.maxLag = math.max(res.maxLag, fromRef)
        local spd = math.sqrt(sample.v.x ^ 2 + sample.v.z ^ 2)
        res.peakSpeed = math.max(res.peakSpeed or 0, spd)
        if R.descending then res.minDescentAgl = math.min(res.minDescentAgl or 1e9, agl) end        -- lowest height reached before the landing phase
        if (nv.coastSpeed or 0) > 0 and not coast.done and not R.descending and R.phase == "move" then
          if not coast.on then
            if spd >= nv.coastSpeed and (R.ax * sample.v.x + R.az * sample.v.z) > -0.05 and at.tilt < 5 then
              coast.on, coast.t0, coast.w0 = true, now, nil
              say(string.format("drag measurement: coasting level from %.0f m/s", spd))
            end
          else
            -- the reference stays with the ship while it coasts (otherwise it runs on ahead and the ship has to chase it afterwards)
            if R.leg then
              local g = R.leg
              g.s = M.clamp((sample.p.x - g.x0) * g.ux + (sample.p.z - g.z0) * g.uz, 0, g.L)
              g.v = math.max(0, sample.v.x * g.ux + sample.v.z * g.uz)
              R.x, R.z = g.x0 + g.ux * g.s, g.z0 + g.uz * g.s
              R.vx, R.vz, R.ax, R.az = g.ux * g.v, g.uz * g.v, 0, 0
              ref.px, ref.pz, ref.vx, ref.vz, ref.ax, ref.az = R.x, R.z, R.vx, R.vz, 0, 0
            end
            -- the measuring window starts once the ship is level (the lean of the acceleration is gone) and ends after coastSecs
            if not coast.w0 and at.tilt < 2 then coast.w0 = { t = now, v = spd, p = pScale } end
            if now - coast.t0 >= (nv.coastSecs or 4) or at.tilt > 12 or sample.p.y < ref.y - 6 then
              coast.on, coast.done = false, true
              local w = coast.w0
              local T = w and (now - w.t) or 0
              if w and T > 1 and spd > 1 and w.v > spd then
                local k = (1 / spd - 1 / w.v) / T                 -- dv/dt = -k v^2  ->  1/v1 - 1/v0 = k t
                local pr = math.max(0.2, 0.5 * (w.p + pScale))
                res.coast = { v0 = w.v, v1 = spd, t = T, k = k, kGround = k / pr, pressure = pr }
                local a = (nv.coneAccel or 1.9) + 0.6 * (nv.gravity or 11) * math.tan(math.rad(3))
                say(string.format("drag measurement: %.1f -> %.1f m/s in %.1f s: k %.2e per m at air %.0f%% (%.2e at the ground); top speed estimate with %.1f m/s2 of thrust: %.0f m/s",
                  w.v, spd, T, k, 100 * pr, k / pr, a, math.sqrt(a / k)))
              else
                say("drag measurement: no usable result")
              end
            end
          end
        end
        if floorY and not R.descending and R.phase ~= "alt" then
          local clear = sample.p.y - floorY
          if clear < -(opts.floorMargin or 1.5) then              -- below the floor: climb and slow down
            R.y = math.max(R.y, floorY + 3)
            if R.leg then R.leg.v = R.leg.v * 0.9 end
            res.floorBreaches = (res.floorBreaches or 0) + 1
          end
          res.minClear = math.min(res.minClear or 1e9, clear)
        end
        -- lean governor (only when the route allows lean at all)
        ref.leanLimit = 0
        if (nv.leanMax or 0) > 0 then
          local clear = floorY and (sample.p.y - floorY) or nil
          lean.peakY = math.max(lean.peakY or sample.p.y, sample.p.y)
          -- sag measured from the highest altitude reached (a ship still climbing has no sag), but never from above the altitude reference: a climb that
          -- overshoots the cruise height by a metre or two and settles must not look like a sag for the rest of the flight
          local errAlt = math.max(0, math.min(lean.peakY, ref.y + 0.3) - sample.p.y)
          if R.descending then errAlt = ref.y - sample.p.y end   -- the final approach lowers the goal on purpose: sag is measured from the descent reference then
          if not lean.armed and (not clear or clear > 2) then lean.armed = true end
          if lean.armed and (errAlt > (nv.leanLockErr or 3) or (clear and clear < 1 and not R.descending)) then lean.locked = true end
          if R.descending and agl < (nv.leanDescentMin or 10) then lean.locked = true end
          if lean.armed and not lean.locked then
            local braking = (R.ax * sample.v.x + R.az * sample.v.z) < 0
            ref.leanLimit = Nav.leanLimit(nv.leanMax, spd, lean.maxFrac, errAlt, clear, nv, braking)
          end
          res.maxLean = math.max(res.maxLean or 0, at.tilt)
        end
        if R.phase == "dwell" and R.i == #R.route and R.dwellEnd and now - (R.dwellEnd - route[#route].dwell) > 3 then
          -- hold quality at the last waypoint (after 3 s of settling)
          res.holdErr = res.holdErr + (sample.p.y - ref.y) ^ 2
          res.holdN = res.holdN + 1
        end
        if opts.hook and R.phase == "dwell" and R.i == #R.route then
          local ov = opts.hook({ t = now - t0, dwell = now - (R.dwellEnd - route[#route].dwell), st = st, ref = ref, ctl = ctl, res = res, wFd = wFd, sample = sample })
          if ov then
            cref = {}
            for k, v in pairs(ref) do cref[k] = v end
            if ov.openLoop then cref.px, cref.pz, cref.ax, cref.az = nil, nil, ov.ax or 0, ov.az or 0 end
            cref.probe, cref.freeze, cref.leanTest = ov.probe, ov.freeze, ov.leanTest
            if ov.finish then R.dwellEnd = now end
          end
        end
        -- arrival that never completes: the ship sits still near the last waypoint but not within the arrival distance (a position loop that cannot close
        -- the last metres, e.g. a lean-only axis): after 12 s the flight counts as arrived (the final error is in the report) instead of waiting for the time limit
        if R.i == #R.route and R.phase == "move" and R.leg and R.leg.s >= R.leg.L - 1e-6 then
          local lw = R.route[#R.route]
          local dd = math.sqrt((sample.p.x - lw.x) ^ 2 + (sample.p.z - lw.z) ^ 2)
          if dd < 15 and spd < 0.5 and math.abs(sample.p.y - lw.y) < 2 then stuckT2 = stuckT2 + dt else stuckT2 = 0 end
          if stuckT2 > 12 then
            R.phase = "dwell"
            R.dwellEnd = now + lw.dwell
            R.log[#R.log + 1] = { i = R.i, err = dd, t = now }
            R.vx, R.vz, R.ax, R.az = 0, 0, 0, 0
            res.arriveNote = string.format("arrival accepted %.1f m from the waypoint (the position loop did not close the last metres)", dd)
            say(res.arriveNote)
          end
        end
        if R.done then
          phase, tPhase = "land", now; landRef = { x = ref.px, z = ref.pz }; say("landing")
          if res.holdN > 60 then trimSnap = Ctl.getTrim(ctl) end       -- the integrators after a settled hover
        end
        if phase == "route" and now - routeStart > expect + (opts.slack or 60) then
          phase, tPhase = "land", now; landRef = { x = ref.px, z = ref.pz }; res.ok = false; res.reason = "time limit"; say("time limit: landing")
        end
      elseif phase == "land" then
        local r = (agl < 1.5) and 0.25 or (agl > 15 and math.min(3, landRate + 0.05 * (agl - 15)) or landRate)      -- a long way down goes faster
        ref.y = math.max(ground - 1, ref.y - r * dt)
        ref.vy = -r
        ref.px, ref.pz, ref.vx, ref.vz, ref.ax, ref.az = landRef.x, landRef.z, 0, 0, 0, 0
        local lowAndStill = agl < 0.5 and math.abs(sample.v.y) < 0.05
        local contact = agl < 1.5 and ref.y < sample.p.y - 0.35 and math.abs(sample.v.y) < 0.05      -- the ground holds the ship up while the reference sinks
        if agl < 0.12 or lowAndStill or contact then stoppedT = stoppedT + dt else stoppedT = 0 end
        if agl < 0.12 or stoppedT > (contact and 1.0 or 0.7) then res.landedAgl = agl; hal.stop(); break end
        if now - tPhase > (topY - ground) / landRate + 40 then cut("landing timeout") break end
      end
      if fromRef > maxDrift and phase == "route" then phase, tPhase = "land", now; landRef = { x = sample.p.x, z = sample.p.z }; res.ok = false; res.reason = "drift"; say("drift fault: landing") end
      if agl > (topY - ground) + 6 and phase ~= "land" then phase, tPhase = "land", now; landRef = { x = sample.p.x, z = sample.p.z }; res.ok = false; res.reason = "too high"; say("altitude fault: landing") end
      -- heading reference: slews toward the target (a turn that keeps a weak side off the travel axis) at yawRate deg/s
      local dpsi = M.wrap180(psiTarget - psiNowRef)
      local stepP = (opts.yawRate or 35) * dt
      psiNowRef = M.wrap180(psiNowRef + M.clamp(dpsi, -stepP, stepP))
      if R.hold and math.abs(M.wrap180(at.psi - psiTarget)) < 8 and math.abs(dpsi) < 1 then R.hold = false end
      ref.psi, cref.psi = psiNowRef, psiNowRef
      if levelMode or coast.on then
        cref = {}
        for k, v in pairs(ref) do cref[k] = v end
        cref.px, cref.pz, cref.vx, cref.vz, cref.ax, cref.az, cref.leanLimit = nil, nil, nil, nil, 0, 0, 0
        if levelMode then cref.vy = math.min(cref.vy or 0, 0) end
      end
      local T, tilt, info = Ctl.step(ctl, st, cref, dt)
      rpm, targets = {}, {}
      -- thin air gives less thrust per RPM: scale the commands with the air pressure relative to the ground (v1 hover RPM ~ 1 / pressure)
      if sample.pressure then
        P0 = P0 or sample.pressure
        if P0 > 0 then pScale = M.clamp(sample.pressure / P0, 0.3, 1.3) end
        res.minPressureScale = math.min(res.minPressureScale or 1, pScale)
      end
      for j = 1, n do
        rpm[j] = M.clamp(A.rpmFor(af, j, T[j]) / pScale, 0, cap)
        targets[j] = norm({ x = tilt[j][1], y = 1, z = tilt[j][2] })
      end
      if opts.rpmHook then rpm = opts.rpmHook(rpm, now - t0, phase) or rpm end       -- hidden imbalances for the imbal test
      local mx = 0
      for j = 1, n do mx = math.max(mx, rpm[j]) end
      -- the headroom the lean governor sees is the SUSTAINED rpm demand (0.6 s average): torque transients that touch the limit for a moment must not
      -- switch the lean off (that made a heavy ship oscillate between lean and no lean)
      lean.maxFrac = lean.maxFrac + (mx / cap - lean.maxFrac) * math.min(1, dt / 0.6)
      if opts.log then opts.log(now - t0, st, rpm, phase, info, { fd = wB, apiW = apiW, apiB = apiB, q = q, ref = ref, R = R }) end
    end
    res.rows = res.rows + 1
    res.t = now - t0
    sample = hal.cycle(rpm, targets)
    if opts.maxSeconds and now - t0 > opts.maxSeconds then cut("time limit") break end
  end
  local best, bs = nil, -2
  for k, d in pairs(gyro.dot) do local sc = d / math.max(gyro.den[k], 1e-9) if sc > bs then best, bs = k, sc end end
  res.gyro, res.gyroScore = best, bs
  for _, w in ipairs(res.waypoints) do w.t = w.t - t0 end      -- report flight time, not the clock
  if trimOn and trimSnap and res.ok then
    local okS = A.saveTrim(af, trimSnap, opts.trimPath)
    res.trimSaved = okS
  end
  res.pressureScale = pScale
  res.brakeScale = R.brakeScale
  res.trim = Ctl.getTrim(ctl)
  if res.holdN > 0 then res.holdRms = math.sqrt(res.holdErr / res.holdN) end
  res.final = { x = sample.p.x, y = sample.p.y, z = sample.p.z }
  res.finalPsi = M.attitude(sample.q).psi
  if fs.exists(lockPath) and (not F.testMode or opts.lockPath) then fs.delete(lockPath) end
  if res.ok and res.reason == nil then res.reason = "ok" end
  return res
end

-- Largest horizontal distance between two rotors (m): a lower bound for the size of the ship
function F.extent(af)
  local e = 0
  for i = 1, #af.rotors do
    for j = i + 1, #af.rotors do
      local a, b = af.rotors[i], af.rotors[j]
      e = math.max(e, math.sqrt((a.x - b.x) ^ 2 + (a.z - b.z) ^ 2))
    end
  end
  return e
end

-- Fast point-to-point flight: climb above the safety floor (leaving as soon as the ship is above it), cruise at cruiseAlt with the speed-dependent
-- acceleration plan, lean the whole ship for extra acceleration when p.lean > 0, then descend from the ship's own state in the final approach.
-- (the final approach stops 2.5 m or more above the start height, not 1 m: the descent from the cruise height overshoots its target by about a metre
-- at 5-6 m/s and touched the ground at speed; the landing phase takes the last metres at 0.6 m/s)
-- p: {X, Z, cruise (m above the start), vmax, accel, floor (m above the start), lean (deg), dragK}
-- Cruise height (m above the start) for a transit of `dist` metres. Higher air is thinner: the drag falls with the air pressure, while the rotors
-- need more RPM for the same thrust (rpm ~ 1 / pressure), so the lean the motors can still afford shrinks. For every candidate height this predicts
-- the time of the whole trip (climb + cruise at the speed where the lean thrust balances the drag + descent) and returns the best one.
-- hal.pressureAt(x, y, z) gives the real pressure profile. Returns height, info {pressure ratio, rpm fraction, lean (deg), speed, time}.
function F.bestCruise(hal, af, s, floor, dist, opt)
  opt = opt or {}
  local lowH = floor + 4
  local P0 = hal.pressureAt and hal.pressureAt(s.p.x, s.p.y, s.p.z)
  local ctl = Ctl.new(af, { g = af.g })
  if not (P0 and P0 > 0 and ctl) then return lowH, { why = "no pressure data: low cruise" } end
  local T0 = Al.thrust(ctl.al, af.mass * af.g, 0, 0)
  local capR = af.motorCap or 250
  local hoverFrac = 0
  for j = 1, #af.rotors do hoverFrac = math.max(hoverFrac, A.rpmFor(af, j, T0[j]) / capR) end
  local leanMax = opt.leanMax or 0
  local g = af.g or 11
  local k0 = opt.dragK or 0.0002
  local cone = opt.coneAccel or 1.9
  local vmax = opt.vmax or 40
  local climb, desc = opt.climb or 4, opt.descent or 4.5
  local best
  for h = lowH, opt.maxH or 260, 6 do
    local P = hal.pressureAt(s.p.x, s.p.y + h, s.p.z)
    if not P then break end
    local pr = P / P0
    if pr < 0.4 then break end
    local fr = hoverFrac / pr                              -- rpm fraction needed just to hover up there
    if fr > 0.8 then break end
    local lean = 0
    if leanMax > 0 then lean = math.min(leanMax, math.acos(math.min(1, fr / 0.9)) * M.DEG) end
    local a = cone + 0.6 * g * math.tan(lean / M.DEG)       -- lateral acceleration capability (cone + derated lean)
    local vt = math.sqrt(a / (k0 * pr))                     -- terminal speed: thrust balances the drag (drag ~ pressure)
    local vc = math.min(vmax, 0.9 * vt)
    local t = h / climb + h / desc + dist / vc + 2 * vc / a
    if not best or t < best.t then best = { h = h, t = t, pr = pr, fr = fr, lean = lean, v = vc } end
  end
  if not best then return lowH, { why = "no thrust headroom above the floor" } end
  return best.h, best
end

function F.transit(hal, af, p, opts)
  opts = opts or {}
  local s = hal.sense()
  local ext = F.extent(af)
  local floor = p.floor or math.max(8, math.ceil(ext * 1.25 + 4))
  local vmax = p.vmax or 20
  -- lateral authority along the travel direction at the heading the ship will fly it (a nose-first ship faces the travel direction: its strong axis)
  local legCap
  local dist = math.sqrt((p.X - s.p.x) ^ 2 + (p.Z - s.p.z) ^ 2)
  do
    local lat = af.tune and af.tune.lat
    if lat and dist > 1 then
      local _, _, c = F.pickHeading(af, opts, (p.X - s.p.x) / dist, (p.Z - s.p.z) / dist, M.attitude(s.q).psi)
      legCap = c
    elseif lat then
      legCap = math.min(lat.xp or 1, lat.xn or 1, lat.zp or 1, lat.zn or 1)
    end
  end
  local coneAccel = 1.9 * math.min(1, legCap or 1)
  local dragK0 = p.dragK or (af.tune and af.tune.dragK) or 0.0002
  local cruise, cinfo = p.cruise, nil
  if not cruise then
    cruise, cinfo = F.bestCruise(hal, af, s, floor, dist, { leanMax = p.lean or 0, vmax = vmax, dragK = dragK0, coneAccel = coneAccel, climb = p.climb })
    if opts.say and cinfo and cinfo.pr then
      opts.say(string.format("cruise height %d m above the start (air %.0f%% of the ground pressure, hover needs %.0f%% of the motor RPM, lean up to %.0f deg, about %.0f m/s)",
        cruise, 100 * cinfo.pr, 100 * cinfo.fr, cinfo.lean, cinfo.v))
    end
  end
  local prC = 1                                            -- pressure at the cruise height relative to the ground: drag scales with it
  if hal.pressureAt then
    local P0 = hal.pressureAt(s.p.x, s.p.y, s.p.z)
    local Pc = hal.pressureAt(s.p.x, s.p.y + cruise, s.p.z)
    if P0 and Pc and P0 > 0 then prC = math.max(0.2, Pc / P0) end
  end
  local o = {}
  for k, v in pairs(opts) do o[k] = v end
  o.floor = floor
  o.slack = o.slack or 120
  o.nav = {
    plan = true, speed = vmax, maxSpeed = vmax, accel = p.accel or 2.0, climbRate = p.climb or ((cruise > 50) and 5 or 3), vertAccel = 4, descentRate = (cruise > 50) and 6.5 or 4.5, maxDescent = (cruise > 50) and 6.5 or 4.5,
    maxLag = 30, dwell = 0.5, altTol = 1.0, arriveDist = 1.0, arriveSpeed = 0.6, maxAgl = math.max(60, cruise + 5), legStartY = s.p.y + math.max(floor + 2, 0.85 * cruise),
    leanMax = p.lean or 0, dragK = dragK0 * prC, gravity = af.g,
    capScaleMax = p.capMax or 1.8,                           -- the acceleration plan probes up to this multiple of the modelled capability while the ship keeps up
    dragBrake = p.dragBrake or 0.6,                          -- the drag helps the braking (a share of it: the drag estimate is rough)
    coneAccel = coneAccel,
    planAccelMin = 0.6, leanRpmCap = 0.9, leanRpmBand = 0.1,
    brakeScale0 = af.tune and af.tune.planBrake or 0.6,        -- learned braking capability (ratio of achieved to planned deceleration)
  }
  local infl = p.inflow or (af.tune and af.tune.leanInflow)
  if infl then o.nav.leanInflow = infl end
  o.ctl = o.ctl or {}
  o.ctl.coneAccel = o.nav.coneAccel
  o.ctl.kiAlt = o.ctl.kiAlt or 0.25                      -- the lift lost to the airflow at speed is learned faster in a transit
  o.maxTilt = o.maxTilt or 45
  return F.route(hal, af, { { x = p.X, z = p.Z, y = s.p.y + cruise, speed = vmax, dwell = 0.5, descend = s.p.y + (p.finalHeight or math.max(2.5, 0.15 * ext)) } }, o), { floor = floor, cruise = cruise }
end

-- After a restart in mid-air: hover where the ship is (1 m higher), then land under control on the recorded ground height.
function F.recover(hal, af, opts)
  opts = opts or {}
  local lf = fs.exists(opts.lockPath or F.LOCK) and fs.open(opts.lockPath or F.LOCK, "r")
  if not lf then return { ok = false, reason = "no flight lock" } end
  local ok, lock = pcall(textutils.unserialiseJSON, lf.readAll())
  lf.close()
  if not ok or type(lock) ~= "table" or not lock.ground then return { ok = false, reason = "bad flight lock" } end
  if lock.orig and hal.orig then hal.orig = lock.orig end        -- the HAL was created mid-flight: its own 'original' speeds are not idle
  local s = hal.sense()
  local o = {}
  for k, v in pairs(opts) do o[k] = v end
  o.airborne = true
  o.ground = lock.ground
  o.trim = false
  o.maxDrift = o.maxDrift or 20
  o.nav = o.nav or { minAgl = 0.5, maxAgl = 400, dwell = 0.5, altTol = 1.0 }
  if s.p.y - lock.ground < 0.3 then
    hal.stop()
    if fs.exists(opts.lockPath or F.LOCK) then fs.delete(opts.lockPath or F.LOCK) end
    return { ok = true, reason = "already on the ground" }
  end
  return F.route(hal, af, { { x = s.p.x, z = s.p.z, y = s.p.y + 1, dwell = 0.5 } }, o)
end

function F.hover(hal, af, opts)
  opts = opts or {}
  local s = hal.sense()
  return F.route(hal, af, { { x = s.p.x, z = s.p.z, y = s.p.y + (opts.height or 5), dwell = opts.secs or 10 } }, opts)
end

return F
