-- ap/lib/nav.lua : route planning and the reference generator (pure logic, selftested). Ported from the quadcopter's Traj/route code.
-- World axes: +X east, +Z south, +Y up; compass 0 = north (-Z), 90 = east (+X).
-- Two modes share the same follower:
--   plain routes  : constant acceleration, small speeds (goto / move / wp / cruise)
--   transit       : cfg.plan = true. Speed-dependent acceleration and braking capability (thrust cone + whole-ship lean, drag), a stopping
--                   table from the braking capability, a long lag cap, a safety floor and a final-approach descent from the ship's state.
local M = require("m")
local N = {}

N.defaults = {
  speed = 5, maxSpeed = 8, accel = 1.5, climbRate = 0.8, descentRate = 0.6, vertAccel = 2, dwell = 2,
  maxLag = 3,                 -- m: the reference never gets farther than this ahead of the ship
  arriveDist = 0.6, arriveSpeed = 0.4, altTol = 0.4,
  minAgl = 1, maxAgl = 60,   -- allowed waypoint altitude range above the start
  -- speed-dependent capability (transit only)
  coneAccel = 1.9, dragK = 0, dragBrake = 0, leanMax = 0, leanInflow = 2.0, leanDerate = 0.6, leanBrakeMax = 25, leanBrakeInflow = 14,
  gravity = 11, planAccelMin = 0.6, maxPlanAccel = 6,
  capScaleMax = 1, capProbe = 0.03,       -- the acceleration plan may probe up to capScaleMax x the modelled capability (transit sets 1.8)
  brakeLearnDelay = 0.8, brakeRecover = 0.03,
  coastSpeed = 0, coastSecs = 4,          -- drag measurement: coast level for coastSecs once the ground speed passes coastSpeed (0 = off)
  -- final approach (transit only)
  descendLead = 12, descendRadius = 140, descendMaxSpeed = 20, touchRate = 0.4, flareAlt = 0.6, descentGain = 1.5, maxDescent = 4.5,
}

local function clamp(v, lo, hi) return M.clamp(v, lo, hi) end

-- Planned lateral acceleration capability (m/s^2) at ground speed v: the thrust cone minus drag plus a derated share of the lean the
-- governor allows at that speed (the inflow budget limits it: asin(leanInflow / v)).
function N.accelAt(cfg)
  return function(v)
    local lean = math.min(cfg.leanMax, math.asin(math.min(1, cfg.leanInflow / math.max(v, 0.5))) * M.DEG)
    local a = cfg.coneAccel - cfg.dragK * v * v + cfg.leanDerate * cfg.gravity * math.tan(lean / M.DEG)
    return clamp(a, cfg.planAccelMin, cfg.maxPlanAccel)
  end
end

-- Planned BRAKING capability: the cone plus drag (helps) plus the derated lean share (leaning back into the wind gains thrust, so the budget is larger).
function N.decelAt(cfg)
  return function(v)
    local lean = math.min(cfg.leanBrakeMax, math.asin(math.min(1, cfg.leanBrakeInflow / math.max(v, 0.5))) * M.DEG)
    if cfg.leanMax <= 0 then lean = 0 end
    local a = cfg.coneAccel + cfg.dragK * v * v * cfg.dragBrake + cfg.leanDerate * cfg.gravity * math.tan(lean / M.DEG)
    return clamp(a, 0.6, cfg.maxPlanAccel)
  end
end

-- Stopping-distance table for a speed-dependent deceleration function (80 % of the capability): D(v) = integral v dv / (0.8 a(v)).
function N.stopTable(aFn, vmax)
  local T, v, d, dv = { v = { 0 }, d = { 0 } }, 0, 0, 0.25
  while v < vmax do
    d = d + (v + dv / 2) * dv / (0.8 * aFn(v + dv / 2))
    v = v + dv
    T.v[#T.v + 1], T.d[#T.d + 1] = v, d
  end
  return T
end

-- Highest speed from which the profile can still stop within dist.
function N.vForDist(T, dist)
  if dist <= 0 then return 0 end
  local n = #T.d
  if dist >= T.d[n] then return T.v[n] end
  for i = 2, n do
    if dist <= T.d[i] then
      local f = (dist - T.d[i - 1]) / (T.d[i] - T.d[i - 1])
      return T.v[i - 1] + f * (T.v[i] - T.v[i - 1])
    end
  end
  return T.v[n]
end

-- Distance the stopping table needs to stop from speed v.
function N.distForV(T, v)
  local n = #T.v
  if v <= 0 then return 0 end
  if v >= T.v[n] then return T.d[n] end
  for i = 2, n do
    if v <= T.v[i] then
      local f = (v - T.v[i - 1]) / (T.v[i] - T.v[i - 1])
      return T.d[i - 1] + f * (T.d[i] - T.d[i - 1])
    end
  end
  return T.d[n]
end

-- 1-D reference generator along a leg: moves s toward goal with speed limit vmax and acceleration limit amax and brakes (at 80 % of
-- amax, so discretisation never makes it arrive late) to stop on the goal. Optional speed-dependent functions aFn (acceleration),
-- dFn (braking) and the stopping table stopT. Returns s, v, acceleration used (feed-forward).
function N.step(s, v, goal, vmax, amax, dt, aFn, stopT, dFn, dscale)
  local d = goal - s
  local dr = (d >= 0) and 1 or -1
  if aFn then amax = aFn(math.abs(v)) end
  if math.abs(d) < 0.005 and math.abs(v) <= 0.15 then return goal, 0, clamp(-v / dt, -amax, amax) end
  local vstop = (aFn and stopT) and N.vForDist(stopT, math.abs(d) * (dscale or 1)) or math.sqrt(2 * 0.8 * amax * math.abs(d))
  local vdes = dr * math.min(vmax, vstop)
  if dFn and math.abs(vdes) < math.abs(v) then amax = dFn(math.abs(v)) end
  local nv = v + clamp(vdes - v, -amax * dt, amax * dt)
  local ns = s + nv * dt
  if (goal - ns) * dr < 0 then ns, nv = goal, 0 end          -- would cross the goal: land on it
  return ns, nv, clamp((nv - v) / dt, -amax, amax)
end

-- Descent speed (m/s, positive) allowed at this height above the final altitude.
function N.rateDown(h, cfg)
  return clamp(cfg.touchRate + (h - cfg.flareAlt) * cfg.descentGain, cfg.touchRate, cfg.maxDescent)
end

-- Final-approach descent may begin only when the SHIP (not the reference, which runs ahead when the ship lags) is inside the descent
-- radius and its braking distance + lead, is slow enough and the leg is past its half way point.
function N.descentStart(remShip, vShip, legS, legL, cfg)
  return remShip < math.min(vShip * vShip / (2 * cfg.accel) + cfg.descendLead, cfg.descendRadius)
    and vShip < cfg.descendMaxSpeed and legS > legL / 2
end

-- Lean (deg) the governor allows for a wanted lean `ang`. speed: ground speed; maxFrac: largest motor command as a fraction of the cap;
-- errAlt: altitude goal minus altitude (positive = sagging); clearance: height above the safety floor (nil = no floor).
-- Leaning the whole ship costs lift in proportion to the air flowing along the propellers (inflow = speed * sin(lean)), so the lean
-- shrinks with speed, with motor headroom, with altitude sag and with the clearance above the floor.
function N.leanLimit(ang, speed, maxFrac, errAlt, clearance, cfg, braking)
  local lim
  if braking and (cfg.leanBrakeMax or 0) > 0 then
    lim = math.min(cfg.leanBrakeMax, math.asin(math.min(1, cfg.leanBrakeInflow / math.max(speed, 0.5))) * M.DEG)
  else
    lim = math.min(cfg.leanMax, math.asin(math.min(1, cfg.leanInflow / math.max(speed, 0.5))) * M.DEG)
  end
  local capF, bandF = cfg.leanRpmCap or 0.9, cfg.leanRpmBand or 0.1
  local gm = clamp((capF - maxFrac) / bandF, 0, 1)
  local ga = clamp(((cfg.leanAltZero or 2.0) - errAlt) / ((cfg.leanAltZero or 2.0) - (cfg.leanAltFull or 0.5)), 0, 1)
  local gf = clearance and clamp((clearance - (cfg.leanFloorClear or 2.0)) / 2, 0, 1) or 1
  return math.min(ang, lim) * gm * ga * gf
end

-- Heading (deg, as psi) that gives the best lateral authority along a travel direction (ux, uz): the capability along the travel and along the
-- opposite (braking) direction must both be strong, so a weak side of the ship (lat = {xp, xn, zp, zn}, from `ap tune`) is kept off the travel
-- axis. Body +X in world axes is (cos psi, -sin psi), body +Z is (sin psi, cos psi). Returns the new heading, or nil when the present one is good.
function N.headingCap(lat, ux, uz, psi)
  local r = psi / M.DEG
  local bx, bz = ux * math.cos(r) - uz * math.sin(r), ux * math.sin(r) + uz * math.cos(r)       -- travel direction in body axes
  local function along(dx, dz)
    local gx = (dx > 0) and (lat.xp or 1) or (lat.xn or 1)
    local gz = (dz > 0) and (lat.zp or 1) or (lat.zn or 1)
    return dx * dx * gx + dz * dz * gz
  end
  return math.min(along(bx, bz), along(-bx, -bz))
end

-- Heading (psi of body +X) that points the ship's FRONT along the travel direction (ux, uz). front = {off = heading offset of the front axis from +X}
function N.noseHeading(front, ux, uz)
  local phi = -math.atan2(uz, ux) * M.DEG          -- the psi at which body +X would point along (ux, uz)
  return M.wrap180(phi - front.off)
end

function N.bestHeading(lat, ux, uz, psiNow)
  local function cap(psi) return N.headingCap(lat, ux, uz, psi) end
  local now = cap(psiNow)
  local best, bestCap, bestTurn = psiNow, now, 0
  for k = -12, 11 do
    local psi = M.wrap180(psiNow + k * 15)
    local c = cap(psi)
    local turn = math.abs(k * 15)
    if c > bestCap + 0.02 or (math.abs(c - bestCap) <= 0.02 and turn < bestTurn) then best, bestCap, bestTurn = psi, c, turn end
  end
  if bestCap > now * 1.15 and bestTurn >= 12 then return best end
  return nil
end

-- compass bearing -> unit vector (x, z)
function N.compassVec(h)
  local r = h / M.DEG
  return math.sin(r), -math.cos(r)
end

-- Waypoints {x, z, y?, speed?, dwell?} (absolute world coordinates; y absolute, default start + 3) -> validated route.
-- start = {x, y, z}. cfg.maxSpeed bounds the speeds. Returns route, err.
function N.build(points, start, cfg)
  cfg = cfg or N.defaults
  local out = {}
  for i, w in ipairs(points) do
    local x, z = tonumber(w.x or w[1]), tonumber(w.z or w[2])
    if not (x and z) then return nil, "waypoint " .. i .. " needs x and z" end
    local y = tonumber(w.y or w[3]) or (start.y + 3)
    local lo, hi = start.y + (cfg.minAgl or 1), start.y + (cfg.maxAgl or 60)
    if y < lo - 1e-6 or y > hi + 1e-6 then return nil, string.format("waypoint %d altitude %.1f outside %.1f..%.1f (%.0f..%.0f m above the start)", i, y, lo, hi, cfg.minAgl or 1, cfg.maxAgl or 60) end
    local sp = tonumber(w.speed or w[4]) or cfg.speed or N.defaults.speed
    out[i] = { x = x, z = z, y = y, speed = clamp(sp, 0.5, cfg.maxSpeed or N.defaults.maxSpeed), dwell = tonumber(w.dwell or w[5]) or cfg.dwell or N.defaults.dwell, descend = w.descend }
  end
  if #out == 0 then return nil, "no waypoints" end
  return out
end

-- rough duration (s) of a route flown from `start`
function N.estimate(route, start, cfg)
  cfg = cfg or N.defaults
  local t, x, z = 8, start.x, start.z
  for _, w in ipairs(route) do
    local L = math.sqrt((w.x - x) ^ 2 + (w.z - z) ^ 2)
    local a = cfg.accel or N.defaults.accel
    t = t + L / w.speed + 2 * w.speed / a + w.dwell + 3
    x, z = w.x, w.z
  end
  return t
end

-- Follower state. start = {x, y, z}
function N.new(route, start, cfg)
  local c = {}
  for k, v in pairs(N.defaults) do c[k] = v end
  for k, v in pairs(cfg or {}) do c[k] = v end
  local R = {
    route = route, cfg = c, i = 1, phase = "alt",
    x = start.x, z = start.z, y = start.y, vx = 0, vz = 0, ax = 0, az = 0, vy = 0,
    leg = nil, dwellEnd = nil, done = false, log = {}, descending = false, capScale = 1, brakeScale = c.brakeScale0 or 0.6, prevSpd = nil, shipAcc = 0,
  }
  if c.plan then
    R.aFn, R.dFn = N.accelAt(c), N.decelAt(c)
    local vm = c.speed
    for _, w in ipairs(route) do vm = math.max(vm, w.speed) end
    -- the final approach descends below the height where the whole-ship lean is allowed: braking there is the thrust cone alone. The plan
    -- therefore brakes with lean only above descendMaxSpeed and with the cone below it (the descent starts at that speed).
    R.dFn0 = function(v) return clamp(c.coneAccel + c.dragK * v * v * c.dragBrake, 0.6, c.maxPlanAccel) end
    local dLean = R.dFn
    R.dFn = function(v) if v <= c.descendMaxSpeed then return R.dFn0(v) end return dLean(v) end
    R.stopT = N.stopTable(R.dFn, vm + 1)
    R.stopT0 = N.stopTable(R.dFn0, vm + 1)
  end
  return R
end

-- Advance the reference. ship = {x, y, z, vx, vz, vy}. Returns the follower (R.x, R.z, R.vx.. = reference; R.phase, R.i, R.done). Called
-- every control step with the flight time `now` (s).
function N.update(R, dt, ship, now)
  local c = R.cfg
  local wp = R.route[R.i]
  if R.done or not wp then R.done = true return R end

  -- altitude: slew the reference toward the waypoint's altitude (rate limit, acceleration limit, braking before the target)
  local dy = wp.y - R.y
  if math.abs(dy) > 1e-6 then
    local rate
    if dy > 0 then rate = c.climbRate
    elseif R.descending then rate = N.rateDown(R.y - wp.y, c)
    else rate = c.descentRate end
    local vmaxY = math.min(rate, math.sqrt(2 * 0.8 * c.vertAccel * math.abs(dy)))
    local vdes = (dy > 0) and vmaxY or -vmaxY
    local va = c.vertAccel
    R.vy = R.vy + clamp(vdes - R.vy, -va * dt, va * dt)
    local ny = R.y + R.vy * dt
    if (wp.y - ny) * dy <= 0 then ny, R.vy = wp.y, 0 end
    R.y = ny
  else
    R.vy = 0
  end
  local altNear = math.abs(R.y - wp.y) < 1e-3
  local altOk = altNear and math.abs(ship.y - wp.y) < c.altTol

  if R.phase == "alt" then
    -- first get to the waypoint altitude (vertical climb from the takeoff point), then start the leg
    R.vx, R.vz, R.ax, R.az = 0, 0, 0, 0
    local go = altOk
    if c.legStartY then go = ship.y >= c.legStartY and R.y >= c.legStartY end      -- transit: leave while still climbing above the floor
    if go then
      R.phase = "move"
      R.leg = nil
    end
  end
  if R.phase == "move" then
    if not R.leg then
      local dx, dz = wp.x - R.x, wp.z - R.z
      local L = math.sqrt(dx * dx + dz * dz)
      R.leg = { x0 = R.x, z0 = R.z, L = L, ux = L > 1e-6 and dx / L or 0, uz = L > 1e-6 and dz / L or 0, s = 0, v = 0 }
    end
    local g = R.leg
    if not g.announced then
      g.announced = true
      if R.onLegStart then R.onLegStart(R, wp, g.ux, g.uz, g.L) end      -- the flight program may ask the ship to turn first (weak sides)
    end
    if R.hold then
      R.vx, R.vz, R.ax, R.az = 0, 0, 0, 0
      return R
    end
    -- lag cap: when the ship falls behind the reference, the reference waits
    local lag = math.sqrt((R.x - ship.x) ^ 2 + (R.z - ship.z) ^ 2)
    local amax = c.accel
    if lag > c.maxLag and g.s > 0 then
      g.v = math.max(0, g.v - amax * dt)
      R.ax, R.az = -g.ux * amax * 0.5, -g.uz * amax * 0.5
      R.x, R.z = g.x0 + g.ux * g.s, g.z0 + g.uz * g.s
      R.vx, R.vz = g.ux * g.v, g.uz * g.v
      g.s = g.s + g.v * dt
      R.lagged = (R.lagged or 0) + dt
    else
      local aF, dF, vm, ds = R.aFn, R.dFn, wp.speed, 1
      local stopT = R.stopT
      if R.descending and R.dFn0 then dF, stopT = R.dFn0, R.stopT0 end
      if R.aFn then
        -- the plan assumes a capability; the ship may have less. The capability scale drops while the ship lags the reference and creeps
        -- back up when it keeps up, and the speed is also limited by what the SHIP could stop from at its own position.
        local shipS = (ship.x - g.x0) * g.ux + (ship.z - g.z0) * g.uz
        local lagS = g.s - shipS
        -- (thresholds scale with the lag cap: a lean transient early in the leg is not a capability problem)
        local lagHi, lagLo = 0.7 * c.maxLag, 0.35 * c.maxLag
        if lagS > lagLo and R.capScale > 1 then
          -- the probe found the ship's limit: back off at once and do not probe above 90 % of the scale where the lag appeared again
          R.capCeil = math.max(1, math.min(R.capCeil or 9, R.capScale * 0.9))
          R.capScale = math.max(1, R.capScale - 0.6 * dt)
        elseif lagS > lagHi then R.capScale = math.max(0.4, R.capScale - 0.3 * dt * (lagS - lagHi) / lagHi)
        elseif lagS < lagLo then
          -- a ship that keeps up with the plan may be able to do more than the plan assumes (the capability model is conservative): the
          -- acceleration scale probes upward (only the acceleration: braking keeps its own scale) until the ship starts to lag
          local top = math.min(c.capScaleMax or 1, R.capCeil or 9)
          local up = (R.capScale < 1) and 0.15 or (c.capProbe or 0.03)
          R.capScale = math.min(top, R.capScale + up * dt)
        end
        -- braking capability: the plan assumes decel(v); what the ship really manages is measured while it slows down (smoothed) and the
        -- stopping distances use the smaller of the two scales, so an optimistic braking plan can never run the ship past the goal
        local spdNow = math.sqrt(ship.vx ^ 2 + ship.vz ^ 2)
        if R.prevSpd then R.shipAcc = R.shipAcc + 0.15 * (((spdNow - R.prevSpd) / dt) - R.shipAcc) end
        R.prevSpd = spdNow
        local dPlan = dF(math.max(spdNow, 0.5))
        -- the ship must be clearly (and for a while) slower to brake than the reference before the plan is judged too optimistic: the
        -- transition from accelerating to braking always shows a short overshoot that says nothing about the capability. While the ship
        -- keeps up with the reference the scale creeps back up (never above the value the leg started with: that one was measured in earlier
        -- flights), so one early transient cannot make the rest of a long leg crawl.
        if spdNow > 8 and R.shipAcc < -0.5 and g.v < spdNow - 1.5 then
          R.brakeLagT = (R.brakeLagT or 0) + dt
          if R.brakeLagT > (c.brakeLearnDelay or 0.8) then
            local ratio = clamp(-R.shipAcc / dPlan, 0.3, 1.2)
            R.brakeScale = clamp(R.brakeScale + 1.2 * dt * (ratio - R.brakeScale), 0.35, 1)
          end
        else
          R.brakeLagT = math.max(0, (R.brakeLagT or 0) - dt)
          if lagS < lagLo and spdNow > 8 and g.v0 and g.v < g.v0 and spdNow - g.v < 0.8 then R.brakeScale = math.min(c.brakeScale0 or 0.6, R.brakeScale + (c.brakeRecover or 0.03) * dt) end
        end
        g.v0 = g.v
        local sc = R.capScale
        local sb = math.min(math.min(1, sc), R.brakeScale)
        local dBase = dF
        aF = function(vv) return R.aFn(vv) * sc end
        dF = function(vv) return dBase(vv) * sb end
        ds = sb                                   -- the stop table is for the planned capability: with less capability the distance needed grows, i.e. the table is read at d * sb
        vm = math.min(wp.speed, N.vForDist(stopT, math.max(0.5, g.L - shipS) * sb))      -- never 0: a ship that is already at the goal must still let the reference arrive
      end
      local s, v, a = N.step(g.s, g.v, g.L, vm, amax, dt, aF, stopT, dF, ds)
      g.s, g.v = s, v
      R.x, R.z = g.x0 + g.ux * s, g.z0 + g.uz * s
      R.vx, R.vz = g.ux * v, g.uz * v
      R.ax, R.az = g.ux * a, g.uz * a
    end
    -- transit final approach: the descent starts from the ship's own state
    if wp.descend and not R.descending then
      local remShip = math.sqrt((wp.x - ship.x) ^ 2 + (wp.z - ship.z) ^ 2)
      local vShip = math.sqrt(ship.vx ^ 2 + ship.vz ^ 2)
      local sbNow = math.min(math.min(1, R.capScale), R.brakeScale)
      local needD = N.distForV(R.stopT0, vShip) / sbNow       -- cone-only stopping distance with the braking capability learned so far
      if remShip < math.min(needD + c.descendLead, c.descendRadius * 2) and vShip < c.descendMaxSpeed and g.s > g.L / 2 then
        R.descending = true
        wp.y = wp.descend
      end
    end
    local dist = math.sqrt((wp.x - ship.x) ^ 2 + (wp.z - ship.z) ^ 2)
    local spd = math.sqrt(ship.vx ^ 2 + ship.vz ^ 2)
    if g.s >= g.L - 1e-6 and dist < c.arriveDist and spd < c.arriveSpeed and (altOk or (wp.descend and R.descending and math.abs(ship.y - wp.y) < 1.5)) then
      R.phase = "dwell"
      R.dwellEnd = now + wp.dwell
      R.log[#R.log + 1] = { i = R.i, err = dist, t = now }
      R.vx, R.vz, R.ax, R.az = 0, 0, 0, 0
    end
  elseif R.phase == "dwell" then
    R.vx, R.vz, R.ax, R.az = 0, 0, 0, 0
    if now >= R.dwellEnd then
      if R.i >= #R.route then R.done = true
      else R.i, R.phase, R.leg = R.i + 1, "alt", nil end
    end
  end
  return R
end

return N
