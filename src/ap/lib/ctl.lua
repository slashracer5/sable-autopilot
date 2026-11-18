-- ap/lib/ctl.lua : generic hover / position controller for any rotor layout. Gains are derived from the ship's own mass and
-- inertia (a chosen bandwidth), not hand-tuned per ship. Pure logic: state in, rotor commands out (selftested in the sim).
--
--   attitude  alpha_cmd = wn^2 * phi - 2 zeta wn * omega + integral   torque = I * alpha_cmd / authority
--     phi = (dz, 0, -dx): the small rotation (body frame) that takes the body up axis onto the DESIRED thrust axis, with (dx, dz) the
--     desired axis in body axes (world up when level; tilted by the lean when the whole ship leans for more lateral acceleration).
--     `authority` and the bandwidth come from `ap tune` (af.tune), default 1.
--   altitude  a_cmd = wnA^2 e + 2 zetaA wnA (vref - vy) + integral          Fy = m (g + a_cmd) / cos(tilt)
--   yaw       tau_y = Iy (wnY^2 e - 2 zeta wnY wy + integral)
--   position  world acceleration (+ integral) -> split into the whole-ship LEAN (only when ref.leanLimit > 0: the part above coneAccel,
--             at most g tan(leanLimit), slewed at leanRate) and the BEARING TILT share (the rest, cone limited) via the allocator;
--             per-direction lateral authority from `ap tune`.
-- Trim: the integrators (roll, pitch, yaw, altitude, position) are what learns hidden imbalances; C.getTrim / C.setTrim persist them.
local M = require("m")
local A = require("alloc")
local C = {}

C.defaults = {
  wnAtt = 1.1, zetaAtt = 1.0, kiAtt = 0.12,       -- attitude bandwidth (rad/s; ~0.2 s actuator delay), integral rate (x wn^2 per s)
  wnAlt = 1.0, zetaAlt = 1.1, kiAlt = 0.08,
  wnYaw = 0.8, zetaYaw = 1.0, kiYaw = 0.1,
  kpPos = 0.5, kdPos = 1.4, kiPos = 0.02, maxLat = 2.2,   -- m/s^2 per m, per m/s, per m s; lateral accel cap (cone: tan(11.5 deg) g = 2.2)
  coneDeg = 11.5, attIntMax = 0.5, yawIntMax = 0.3, posIntMax = 0.3,
  attBoost = 3, attBoostRate = 0.0175,   -- attitude integrator x3 while the ship is quasi-static (|rate| < 1 deg/s): finds hidden imbalances fast (v1)
  altBoost = 2, altBoostVy = 0.1,        -- altitude integrator x2 while |vy| < 0.1 m/s
  leanOuterRatio = 0.22, leanOnlyMax = 12,   -- position gains and lean cap for ships that can only translate by leaning (tandem / coaxial)
  coneAccel = 1.9,            -- m/s^2 the bearings are asked for before the body starts to lean
  leanRate = 40,              -- deg/s: how fast the lean setpoint may move
  g = 11,
}

-- Largest pure roll / pitch torque (N m) the rotor thrust limits allow at the hover collective (no net lift change). Axes driven by the bearing
-- tilts use the tilt cone times the lever arm.
function C.torqueCapacity(af, al)
  local W = af.mass * (af.g or 11)
  local T0 = A.thrust(al, W, 0, 0)
  local function cap(axis)
    local lo, hi = 0, 1e7
    for _ = 1, 40 do
      local mid = (lo + hi) / 2
      local _, info = A.thrust(al, W, axis == "x" and mid or 0, axis == "z" and mid or 0)
      if info.attScale >= 0.999 and not info.saturated then lo = mid else hi = mid end
    end
    return lo
  end
  local tilt = 0
  for j, r in ipairs(af.rotors) do tilt = tilt + math.abs(r.y) * T0[j] * math.tan((af.coneDeg or 11.5) / M.DEG) end
  local cx = al.rollTilt and tilt or ((al.ax) and cap("x") or nil)
  local cz = al.pitchTilt and tilt or ((al.az) and cap("z") or nil)
  return cx, cz
end

-- af: {mass, inertia={x,y,z}, rotors={{x,y,z,tmax,tmin}}, tune?={gx,gy,gz,delay,lat={xp,xn,zp,zn}}}
function C.new(af, over)
  local cfg = {}
  for k, v in pairs(C.defaults) do cfg[k] = v end
  for k, v in pairs(over or {}) do cfg[k] = v end
  local tune = af.tune or {}
  -- with a measured actuator delay the attitude bandwidth is limited to ~0.25 / delay (phase margin)
  if tune.delay and tune.delay > 0.05 then cfg.wnAtt = math.min(cfg.wnAtt, 0.25 / tune.delay) end
  local al, err = A.new(af.rotors)
  if not al then return nil, err end
  -- Lean slew limit from the ship's own torque capacity: following a ramp of rate r with a critically damped attitude loop needs an angular
  -- acceleration of 2 zeta wn r, and only a share of what the rotors can deliver (their thrust limits) is used, so the lean setpoint of a
  -- heavy ship with little torque (tandem helicopter) is not slewed faster than the ship can follow (it would saturate the rotors).
  local capX, capZ = C.torqueCapacity(af, al)
  if capX and capZ then
    local alpha = math.min(capX * M.clamp(tune.gx or 1, 0.25, 4) / af.inertia.x, capZ * M.clamp(tune.gz or 1, 0.25, 4) / af.inertia.z)
    local rate = 0.4 * alpha / (2 * cfg.zetaAtt * cfg.wnAtt) * M.DEG
    -- only the lean of a TRANSIT (governor active) is rate limited this way; the small leans of position holding keep the fast slew (a slow slew inside
    -- the position loop made the tandem hold oscillate)
    if over == nil or over.leanRateTransit == nil then cfg.leanRateTransit = math.max(3, math.min(cfg.leanRate, rate)) end
    cfg.alphaMax = alpha
  end
  return {
    af = af, cfg = cfg, al = al, tune = tune,
    ix = 0, iz = 0, ia = 0, iy = 0, ipx = 0, ipz = 0,     -- integrators: roll/pitch (rad/s^2), altitude (m/s^2), yaw (rad/s^2), position (m/s^2)
    lastT = nil, tiltTau = { 0, 0 }, lastTau = { 0, 0, 0 },
    leanX = 0, leanZ = 0,                                 -- lean setpoint as a world acceleration (m/s^2)
  }
end

function C.getTrim(c) return { ix = c.ix, iz = c.iz, ia = c.ia, iy = c.iy, ipx = c.ipx, ipz = c.ipz } end
function C.setTrim(c, t)
  c.ix, c.iz, c.ia, c.iy = t.ix or 0, t.iz or 0, t.ia or 0, t.iy or 0
  c.ipx, c.ipz = t.ipx or 0, t.ipz or 0
end

local function authority(tune, k)
  local g = tune[k] or 1
  return M.clamp(g, 0.25, 4)
end

-- s: {y, vy, ux, uz (world up in body axes), w = {x,y,z} body rates rad/s, psi, wy (rad/s), cosTilt, q (quaternion),
--     vx, vz (world), px, pz (world position)}
-- ref: {y=, vy=, psi=, px=, pz=, vx=, vz=, ax=, az=, leanLimit=} (all optional except y)
--   ref.probe = {tx=, tz=, ty=} torque added after the loops (identification), ref.freeze = true holds the previous attitude/yaw
--   torque (open loop) so a probe's response is not fought by the controller.
function C.step(c, s, ref, dt)
  local cfg, af = c.cfg, c.af
  local n = #af.rotors
  local m, g = af.mass, cfg.g
  local I = af.inertia
  local q = M.qnorm(s.q)

  -- lateral acceleration wanted (world axes)
  local lx, lz = ref.ax or 0, ref.az or 0
  -- axes the bearings cannot push along independently (tandem: roll axis, ...): that body acceleration comes ONLY from leaning the ship
  local onlyX, onlyZ = c.al.pitchTilt, c.al.rollTilt
  if ref.px then
    local ex, ez = ref.px - s.px, ref.pz - s.pz
    -- a lean-only axis goes through the (slow) attitude loop: its position integrator must be far slower than the loop (the heli's z hold swung +-3.5 m
    -- with a 20 s period and then parked 4 m off because the wound-up integral cancelled the proportional lean) and smaller
    local kiP, pim = cfg.kiPos, cfg.posIntMax
    if onlyX or onlyZ then kiP, pim = cfg.kiPos * (cfg.leanOnlyKi or 0.1), math.min(cfg.posIntMax, cfg.leanOnlyIntMax or 0.15) end
    if math.abs(ex) < 3 then c.ipx = M.clamp(c.ipx + kiP * ex * dt, -pim, pim) end
    if math.abs(ez) < 3 then c.ipz = M.clamp(c.ipz + kiP * ez * dt, -pim, pim) end
    if onlyX or onlyZ then
      -- per BODY axis: a lean-only axis goes through the attitude loop, so its position bandwidth must stay well below the attitude loop's
      local wo = cfg.leanOuterRatio * cfg.wnAtt
      local kpS, kdS = wo * wo, (cfg.leanOnlyKd or 1.5) * 2.0 * wo
      local eb = M.qrot(M.qconj(q), { x = ex, y = 0, z = ez })
      local vb = M.qrot(M.qconj(q), { x = (ref.vx or 0) - s.vx, y = 0, z = (ref.vz or 0) - s.vz })
      local lbx = (onlyX and kpS or cfg.kpPos) * eb.x + (onlyX and kdS or cfg.kdPos) * vb.x
      local lbz = (onlyZ and kpS or cfg.kpPos) * eb.z + (onlyZ and kdS or cfg.kdPos) * vb.z
      local lw = M.qrot(q, { x = lbx, y = 0, z = lbz })
      lx = lx + lw.x + c.ipx
      lz = lz + lw.z + c.ipz
    else
      lx = lx + cfg.kpPos * ex + cfg.kdPos * ((ref.vx or 0) - s.vx) + c.ipx
      lz = lz + cfg.kpPos * ez + cfg.kdPos * ((ref.vz or 0) - s.vz) + c.ipz
    end
  end
  local limit = ref.leanLimit or 0                       -- deg of whole-ship lean the governor allows right now
  if ref.leanTest then lx, lz, limit = ref.leanTest[1], ref.leanTest[2], 45 end
  local mag = math.sqrt(lx * lx + lz * lz)
  local cap = cfg.maxLat
  if limit > 0 then cap = cfg.coneAccel + g * math.tan(limit / M.DEG) end
  if mag > cap then lx, lz, mag = lx * cap / mag, lz * cap / mag, cap end
  -- lean setpoint: the share above coneAccel (slewed), shrinking with the governor
  local tLx, tLz = 0, 0
  if ref.leanTest then tLx, tLz = lx, lz                       -- lean test: the whole acceleration comes from the lean
  elseif limit > 0 and mag > cfg.coneAccel then
    local extra = mag - cfg.coneAccel
    extra = math.min(extra, g * math.tan(limit / M.DEG))
    tLx, tLz = lx / mag * extra, lz / mag * extra
  end
  if (onlyX or onlyZ) and not ref.leanTest then
    local lb = M.qrot(M.qconj(q), { x = lx, y = 0, z = lz })
    local bx, bz = onlyX and lb.x or 0, onlyZ and lb.z or 0
    local capL = g * math.tan(cfg.leanOnlyMax / M.DEG)
    local bm = math.sqrt(bx * bx + bz * bz)
    if bm > capL then bx, bz = bx * capL / bm, bz * capL / bm end
    local lw = M.qrot(q, { x = bx, y = 0, z = bz })
    tLx, tLz = tLx + lw.x, tLz + lw.z
  end
  local slewRate = (limit > 0 and cfg.leanRateTransit) or cfg.leanRate
  local slew = g * math.tan(slewRate / M.DEG) * dt     -- m/s^2 the setpoint may change per step
  local dx, dz = tLx - c.leanX, tLz - c.leanZ
  local dm = math.sqrt(dx * dx + dz * dz)
  if dm > slew then dx, dz = dx * slew / dm, dz * slew / dm end
  c.leanX, c.leanZ = c.leanX + dx, c.leanZ + dz
  -- desired thrust axis (world) and its body-frame components
  local dw = { x = c.leanX / g, y = 1, z = c.leanZ / g }
  local dn = math.sqrt(dw.x ^ 2 + 1 + dw.z ^ 2)
  dw = { x = dw.x / dn, y = 1 / dn, z = dw.z / dn }
  local db = M.qrot(M.qconj(q), dw)

  -- altitude
  local e = ref.y - s.y
  local aCmd = cfg.wnAlt ^ 2 * e + 2 * cfg.zetaAlt * cfg.wnAlt * ((ref.vy or 0) - s.vy) + c.ia
  local ab = (math.abs(s.vy) < cfg.altBoostVy) and cfg.altBoost or 1
  -- integrate near the target; far away only when it UNWINDS (an integrator wound up at cruise must be able to come back for the descent)
  if math.abs(e) < 2 or e * c.ia < 0 then c.ia = M.clamp(c.ia + ab * cfg.kiAlt * cfg.wnAlt ^ 2 * e * dt, -3, 3) end
  local Fy = m * (g + aCmd) / M.clamp(s.cosTilt, 0.7, 1)
  -- attitude
  local gx, gy, gz = authority(c.tune, "gx"), authority(c.tune, "gy"), authority(c.tune, "gz")
  local tx, tz, ty
  if ref.freeze then
    tx, tz, ty = c.lastTau[1], c.lastTau[2], c.lastTau[3]
  else
    local phx, phz = db.z, -db.x
    local ax = cfg.wnAtt ^ 2 * phx - 2 * cfg.zetaAtt * cfg.wnAtt * s.w.x
    local az = cfg.wnAtt ^ 2 * phz - 2 * cfg.zetaAtt * cfg.wnAtt * s.w.z
    local bx = (math.abs(s.w.x) < cfg.attBoostRate) and cfg.attBoost or 1
    local bz = (math.abs(s.w.z) < cfg.attBoostRate) and cfg.attBoost or 1
    -- anti-windup: while the rotors are saturated (the last allocation could not give the torque asked) an integrator only unwinds, it does not grow
    local frozen = c.satAtt
    if not (frozen and phx * c.ix > 0) then c.ix = M.clamp(c.ix + bx * cfg.kiAtt * cfg.wnAtt ^ 2 * phx * dt, -cfg.attIntMax, cfg.attIntMax) end
    if not (frozen and phz * c.iz > 0) then c.iz = M.clamp(c.iz + bz * cfg.kiAtt * cfg.wnAtt ^ 2 * phz * dt, -cfg.attIntMax, cfg.attIntMax) end
    tx = I.x * (ax + c.ix) / gx - (c.al.rollTilt and 0 or c.tiltTau[1])
    tz = I.z * (az + c.iz) / gz - (c.al.pitchTilt and 0 or c.tiltTau[2])
    -- yaw
    local pe = M.wrap180((ref.psi or s.psi) - s.psi) / M.DEG
    if math.abs(pe) < 0.5 then c.iy = M.clamp(c.iy + cfg.kiYaw * cfg.wnYaw ^ 2 * pe * dt, -cfg.yawIntMax, cfg.yawIntMax) end
    ty = I.y * (cfg.wnYaw ^ 2 * pe - 2 * cfg.zetaYaw * cfg.wnYaw * s.wy + c.iy) / gy
    c.lastTau = { tx, tz, ty }
  end
  local pr = ref.probe
  if pr then tx, tz, ty = tx + (pr.tx or 0), tz + (pr.tz or 0), ty + (pr.ty or 0) end
  if pr and pr.ay then Fy = Fy + m * pr.ay end       -- unmodelled collective disturbance (m/s^2)

  -- the bearings supply what the body's actual lean does not (acceleration the thrust axis gives: g * horizontal / vertical)
  local upW = M.qrot(q, { x = 0, y = 1, z = 0 })
  local leanActX, leanActZ = 0, 0
  if limit > 0 or math.abs(c.leanX) + math.abs(c.leanZ) > 0.01 then
    local uy = math.max(upW.y, 0.5)
    leanActX, leanActZ = g * upW.x / uy, g * upW.z / uy
  end
  local rx, rz = lx - leanActX, lz - leanActZ
  if pr and pr.ax then rx, rz = rx + pr.ax, rz + (pr.az or 0) end       -- unmodelled lateral disturbance (the position loop does not know)
  if ref.leanTest then rx, rz = 0, 0 end                                -- lean test: the bearings stay out of it
  local rm = math.sqrt(rx * rx + rz * rz)
  local bcap = (limit > 0) and cfg.coneAccel * 1.15 or cfg.maxLat
  if rm > bcap then rx, rz = rx * bcap / rm, rz * bcap / rm end
  local fb = M.qrot(M.qconj(q), { x = rx * m, y = 0, z = rz * m })       -- world lateral force -> body axes
  -- per-direction lateral authority (a weak side needs a bigger command for the same acceleration)
  local lat = c.tune.lat
  if lat then
    local gxs = (fb.x >= 0) and (lat.xp or 1) or (lat.xn or 1)
    local gzs = (fb.z >= 0) and (lat.zp or 1) or (lat.zn or 1)
    fb.x, fb.z = fb.x / M.clamp(gxs, 0.3, 2), fb.z / M.clamp(gzs, 0.3, 2)
  end
  -- thrust first (uses the previous tilt torque), then tilts from the thrusts
  local T, info = A.thrust(c.al, Fy, tx, tz)
  c.lastT = T
  c.satAtt = info.attScale < 0.98
  -- lean-only axes get no bearing force request from the position loop
  if onlyX then fb.x = 0 end
  if onlyZ then fb.z = 0 end
  local tilts, tinfo = A.tilts(c.al, T, fb.x, fb.z, ty, math.tan(cfg.coneDeg / M.DEG), tx, tz)
  if tilts then
    c.tiltTau = { c.al.rollTilt and 0 or tinfo.tauX, c.al.pitchTilt and 0 or tinfo.tauZ }
  else
    tilts = {}
    for j = 1, n do tilts[j] = { 0, 0 } end
  end
  if pr and pr.tilt then for j, t in pairs(pr.tilt) do if tilts[j] then tilts[j] = { tilts[j][1] + t[1], tilts[j][2] + t[2] } end end end
  return T, tilts, { sat = info.saturated, attScale = info.attScale, tiltScale = tinfo and tinfo.scale or 0, Fy = Fy, tau = { tx, ty, tz }, lean = { c.leanX, c.leanZ } }
end

-- read a sim state into the controller's state record
function C.fromSim(sim)
  local q = M.qnorm(sim.q)
  local upB = M.qrot(M.qconj(q), { x = 0, y = 1, z = 0 })
  local at = M.attitude(q)
  return {
    y = sim.p.y, vy = sim.v.y, ux = upB.x, uz = upB.z, w = sim.w, psi = at.psi, wy = sim.w.y, cosTilt = at.cosTilt, q = q,
    vx = sim.v.x, vz = sim.v.z, px = sim.p.x, pz = sim.p.z,
  }
end

return C
