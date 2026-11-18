-- apinstall: self-extracting installer for the general multirotor autopilot (ap). Version 1.5.3
local VERSION = "1.5.3"
local FILES = {
  ["ap.lua"] = [[
-- ap : launcher. Usage: ap <command> ...   (see `ap` with no arguments)
package.path = package.path .. ";/ap/lib/?.lua;/ap/?.lua"
local f = assert(loadfile("/ap/main.lua", "t", _ENV))
return f(...)
]],
  ["ap/lib/airframe.lua"] = [[
-- ap/lib/airframe.lua : the per-ship description (written by the wizard, read by every flight program).
--   af = { version, mass, g, inertia={x,y,z}, motorCap, coneDeg,
--          rotors = { {zone, bearing, motor, trans?, x, y, z, kT, level?, handed, tmax, tmin} ... } }
-- kT = thrust per bearing RPM (measured by the wizard). level = fixed transmission level (nil = direct drive).
local A = {}

A.PATH = "/ap_data/airframe.json"

function A.save(af, path)
  path = path or A.PATH
  local dir = fs.getDir(path)
  if dir ~= "" and not fs.exists(dir) then fs.makeDir(dir) end
  local f = fs.open(path, "w")
  if not f then return false, "cannot write " .. path end
  f.write(textutils.serialiseJSON(af))
  f.close()
  return true
end

function A.load(path)
  path = path or A.PATH
  if not fs.exists(path) then return nil, "no airframe file (" .. path .. "): run `ap wizard`" end
  local f = fs.open(path, "r")
  local s = f.readAll()
  f.close()
  local ok, t = pcall(textutils.unserialiseJSON, s)
  if not ok or type(t) ~= "table" or not t.rotors then return nil, "airframe file unreadable" end
  return t
end

-- motor RPM that makes rotor j deliver thrust T (bearing RPM = T / kT; the transmission scales motor RPM by level/15)
function A.rpmFor(af, j, T)
  local r = af.rotors[j]
  local b = math.max(T, 0) / r.kT
  if r.level and r.level > 0 then b = b * 15 / r.level end
  return b
end

-- thrust of rotor j at motor RPM m
function A.thrustAt(af, j, m)
  local r = af.rotors[j]
  local b = m
  if r.level then b = m * r.level / 15 end
  return r.kT * b
end

-- Pick transmission levels and thrust limits. Hover motor RPM should sit near 50 % of the motor cap so there is room both ways.
-- hoverShare[j]: fraction of the weight rotor j carries in hover (from the allocator). Returns ok, message.
function A.plan(af, hoverShare)
  local cap = af.motorCap or 250
  local W = af.mass * af.g
  local msgs = {}
  for j, r in ipairs(af.rotors) do
    local Th = W * (hoverShare and hoverShare[j] or 1 / #af.rotors)
    local bearingHover = Th / r.kT
    if r.hasTrans then
      local lv = math.ceil(15 * bearingHover / (0.5 * cap))
      lv = math.max(1, math.min(15, lv))
      r.level = lv
    else
      r.level = nil
    end
    local motorHover = A.rpmFor(af, j, Th)
    local tmax = A.thrustAt(af, j, cap)
    r.tmax = tmax
    r.tmin = 0
    r.hoverRpm = motorHover
    if motorHover > 0.8 * cap then
      return false, string.format("%s needs motor RPM %.0f to hover (cap %d): too heavy for its propeller%s", r.zone, motorHover, cap, r.hasTrans and "" or " (no transmission to gear it)")
    end
    if tmax < 1.6 * Th then msgs[#msgs + 1] = string.format("%s: only %.0f%% thrust headroom", r.zone, 100 * (tmax / Th - 1)) end
  end
  return true, table.concat(msgs, "; ")
end

-- Bring a stored airframe up to date with the ship as it is NOW (payload added or removed, centre of mass moved): mass, inertia, gravity and
-- every rotor's arm come from a fresh scan (`info` from hw.scan), the transmission levels and thrust limits are planned again. The measured
-- thrust constants (kT) and the tune stay. Returns ok, message.
function A.refresh(af, info)
  local Al = require("alloc")
  af.mass = info.mass or af.mass
  af.inertia = info.inertia or af.inertia
  af.g = info.g or af.g
  local byName = {}
  for _, r in ipairs(info.rotors) do byName[r.bearing] = r end
  for _, r in ipairs(af.rotors) do
    local n = byName[r.bearing]
    if n and n.pos then r.x, r.y, r.z = n.pos.x, n.pos.y, n.pos.z end
  end
  local al, err = Al.new(af.rotors)
  if not al then return false, err end
  return A.plan(af, al.u)
end

-- FRONT OF THE SHIP -------------------------------------------------------------------------------------------------------------
-- af.front = { axis = "+x"|"-x"|"+z"|"-z" (body axis that points to the front), nose = true|false (fly legs nose-first), src = "fore"|"manual",
--              share = fraction of the weight the front-side rotors carried when it was set }.
-- The default front is the FORE side of the avionics controller: the zone words in the rotor names (fore / aft) were given by Sable relative to
-- the controller, so the direction from the aft rotors to the fore rotors is the controller's forward axis.
local AXV = { ["+x"] = { 1, 0 }, ["-x"] = { -1, 0 }, ["+z"] = { 0, 1 }, ["-z"] = { 0, -1 } }
local OFF = { ["+x"] = 0, ["+z"] = -90, ["-z"] = 90, ["-x"] = 180 }     -- heading offset of the axis from body +x (psi = heading of body +x)
local OPP = { ["+x"] = "-x", ["-x"] = "+x", ["+z"] = "-z", ["-z"] = "+z" }
A.AXES = AXV

local function dominant(dx, dz)
  if math.abs(dx) >= math.abs(dz) then return dx >= 0 and "+x" or "-x" end
  return dz >= 0 and "+z" or "-z"
end

local function zoneMean(rotors, word, notWord)
  local sx, sz, n = 0, 0, 0
  for _, r in ipairs(rotors) do
    local z = r.zone or ""
    if z:find(word, 1, true) and not (notWord and z:find(notWord, 1, true)) then sx, sz, n = sx + r.x, sz + r.z, n + 1 end
  end
  if n == 0 then return nil end
  return sx / n, sz / n
end

-- the controller's forward axis from the zone words; returns axis, reason
function A.defaultFront(rotors)
  local fx, fz = zoneMean(rotors, "fore")
  local ax, az = zoneMean(rotors, "aft")
  local dx, dz
  if fx and ax then dx, dz = fx - ax, fz - az
  elseif fx then dx, dz = fx, fz
  elseif ax then dx, dz = -ax, -az end
  if not dx or math.sqrt(dx * dx + dz * dz) < 0.75 then return "+x", "the rotor names give no fore/aft direction: +x assumed" end
  return dominant(dx, dz), "the fore side of the avionics controller"
end

-- side word -> body axis. fore/aft/port/starboard are relative to the avionics controller; "+x" ... "-z" are body axes. Returns axis or nil, error
function A.resolveSide(af, word)
  word = tostring(word):lower()
  if AXV[word] then return word end
  local fore = A.defaultFront(af.rotors)
  if word == "fore" or word == "front" or word == "forward" then return fore end
  if word == "aft" or word == "rear" or word == "back" then return OPP[fore] end
  if word == "port" or word == "left" or word == "starboard" or word == "right" then
    local px, pz = zoneMean(af.rotors, "port", "starboard")
    local sx, sz = zoneMean(af.rotors, "starboard")
    if not (px and sx) then return nil, "the rotor names do not say where port and starboard are: use +x, -x, +z or -z" end
    local port
    if fore == "+x" or fore == "-x" then port = (pz - sz) >= 0 and "+z" or "-z" else port = (px - sx) >= 0 and "+x" or "-x" end
    if word == "port" or word == "left" then return port end
    return OPP[port]
  end
  return nil, "unknown side '" .. word .. "' (fore, aft, port, starboard, +x, -x, +z, -z)"
end

-- fraction of the hover weight carried by the rotors on the front side of the centre of mass
function A.frontShare(af, axis)
  local Al = require("alloc")
  local al = Al.new(af.rotors)
  if not al then return nil end
  local v, s = AXV[axis], 0
  for j, r in ipairs(af.rotors) do
    if r.x * v[1] + r.z * v[2] > 0.25 then s = s + al.u[j] end
  end
  return s
end

-- elongated ships (tandem helicopters, long planes) fly nose-first by default
local function elongated(af, axis)
  local v = AXV[axis]
  local amin, amax, bmin, bmax = math.huge, -math.huge, math.huge, -math.huge
  for _, r in ipairs(af.rotors) do
    local a, b = r.x * v[1] + r.z * v[2], -r.x * v[2] + r.z * v[1]
    amin, amax, bmin, bmax = math.min(amin, a), math.max(amax, a), math.min(bmin, b), math.max(bmax, b)
  end
  return (amax - amin) >= 1.8 * math.max(bmax - bmin, 2)
end

-- the front in use: {axis, nose, ux, uz (body unit vector), off (heading offset, deg), src, share}
function A.front(af)
  local f = af.front or {}
  local axis = f.axis
  local src = f.src
  if not axis then axis = A.defaultFront(af.rotors); src = "fore" end
  local v = AXV[axis]
  return { axis = axis, nose = f.nose == true, ux = v[1], uz = v[2], off = OFF[axis], src = src, share = f.share }
end

-- choose the front: axis, nose flag (nil = keep / decide from the shape), source label. Stores it in af.front.
function A.setFront(af, axis, nose, src)
  if not AXV[axis] then return nil, "bad axis " .. tostring(axis) end
  local old = af.front or {}
  if nose == nil then
    if old.nose ~= nil then nose = old.nose else nose = elongated(af, axis) end
  end
  af.front = { axis = axis, nose = nose and true or false, src = src or "manual", share = A.frontShare(af, axis) }
  return af.front
end

-- world compass bearing (deg, 0 north, 90 east) of the front axis for the ship orientation q (body -> world quaternion)
function A.frontCompass(af, q)
  local M = require("m")
  local f = A.front(af)
  local w = M.qrot(q, { x = f.ux, y = 0, z = f.uz })
  return (math.atan2(w.x, -w.z) * M.DEG) % 360
end

-- Write a CSV log without ever filling the computer's disk (about 1 MB): at most ~1500 rows (every k-th row is kept), and nothing at all when
-- less than 120 KB are free. Returns true, or false and a reason.
function A.saveLog(path, rows)
  local free = fs.getFreeSpace and fs.getFreeSpace("/") or 1e9
  if free < 120000 then return false, "disk nearly full (run `ap clean`)" end
  local keep = { rows[1] }
  local body = #rows - 1
  local step = math.max(1, math.ceil(body / 1500))
  for i = 2, #rows, step do keep[#keep + 1] = rows[i] end
  local ok, err = pcall(function()
    local dir = fs.getDir(path)
    if dir ~= "" and not fs.exists(dir) then fs.makeDir(dir) end
    local fh = fs.open(path, "w")
    if not fh then error("cannot open " .. path, 0) end
    fh.write(table.concat(keep, "\n"))
    fh.close()
  end)
  if not ok then return false, tostring(err) end
  return true
end

A.TRIM_PATH = "/ap_data/trim.json"

-- Trim (learned integrators) is only valid for the mass it was learned at (within 5 %); returns nil otherwise.
function A.loadTrim(af, path)
  path = path or A.TRIM_PATH
  if not fs.exists(path) then return nil end
  local f = fs.open(path, "r")
  local s = f.readAll()
  f.close()
  local ok, t = pcall(textutils.unserialiseJSON, s)
  if not ok or type(t) ~= "table" or not t.mass then return nil end
  if math.abs(t.mass - af.mass) > 0.05 * af.mass then return nil end
  if t.airframe and af.stamp and t.airframe ~= af.stamp then return nil end
  return t
end

function A.saveTrim(af, trim, path)
  path = path or A.TRIM_PATH
  local dir = fs.getDir(path)
  if dir ~= "" and not fs.exists(dir) then fs.makeDir(dir) end
  local t = {}
  for k, v in pairs(trim) do t[k] = v end
  t.mass = af.mass
  t.airframe = af.stamp
  local f = fs.open(path, "w")
  if not f then return false end
  f.write(textutils.serialiseJSON(t))
  f.close()
  return true
end

return A
]],
  ["ap/lib/alloc.lua"] = [[
-- ap/lib/alloc.lua : control allocation for ANY number of upward-facing rotors at arbitrary positions.
--
-- Body frame: +Y up, rotor j at arm r_j = (x,y,z) from the centre of mass (metres). Thrust T_j along up, with a small tilt
-- (tx_j, tz_j) (tangent of the tilt angle) that the gyroscopic bearing can add. Force F = sum T_j (tx, 1, tz), torque = r x F:
--   tau_x = y Fz - z Fy      tau_y = z Fx - x Fz      tau_z = x Fy - y Fx
--
-- 1. THRUST allocation (collective + roll + pitch): the rows [1; -z; x] map thrusts to (Fy, tau_x, tau_z). T = c*u + tau_x*ax + tau_z*az with
--    u / ax / az the minimum-norm solutions of the unit demands, so u is a PURE collective (no torque) and ax/az are pure torques (no net lift).
--    Limits are met by moving c first (attitude has priority) and only then shrinking the attitude part.
--    A torque row exists only when the rotors are spread along the matching axis. A TANDEM (rotors all on the x axis, z = 0) cannot make roll
--    torque with thrust (rollTilt = true); rotors all on the z axis cannot make pitch torque (pitchTilt = true). Those torques then come from
--    the bearing TILTS (lever arm = height of the rotors above the centre of mass): roll tau_x = sum y T tz, pitch tau_z = -sum y T tx.
-- 2. TILT allocation: unknowns (tx_j, tz_j). Rows: lateral force Fx (unless pitch uses the tx), Fz (unless roll uses the tz), yaw torque
--    tau_y, and the roll / pitch torque of the tilt-driven axes. Minimum-norm solution, capped by the cone angle. The torque the tilts cause on a
--    thrust-driven axis is returned so the thrust allocation can cancel it.
local M = require("m")
local A = {}

local SPREAD = 0.75        -- m: rotors closer than this to an axis count as lying on it
local LEVER = 0.5          -- m: mean height above the centre of mass needed for a tilt-driven torque

-- rotors: array of {x=,y=,z=, tmax=, tmin=}; weights: optional cost weight per rotor (default 1)
function A.new(rotors, weights)
  local n = #rotors
  if n < 2 then return nil, "need at least 2 rotors" end
  local zspread, xspread, ysum = false, false, 0
  for _, r in ipairs(rotors) do
    if math.abs(r.z) > SPREAD then zspread = true end
    if math.abs(r.x) > SPREAD then xspread = true end
    ysum = ysum + math.abs(r.y)
  end
  if not (zspread or xspread) then return nil, "all rotors sit at one point: no lever arm for any torque" end
  local rollTilt, pitchTilt = not zspread, not xspread
  if (rollTilt or pitchTilt) and ysum / n < LEVER then
    return nil, "the rotors are collinear and level with the centre of mass: no way to make roll/pitch torque (needs rotors spread both ways, or above/below the centre of mass)"
  end
  local rows = { function() return 1 end }
  local rollRow, pitchRow
  if zspread then rows[#rows + 1] = function(r) return -r.z end; rollRow = #rows end
  if xspread then rows[#rows + 1] = function(r) return r.x end; pitchRow = #rows end
  if #rows > n then return nil, "not enough rotors to control lift and the torques their spread allows" end
  local B = M.mat(#rows, n, function(i, j) return rows[i](rotors[j]) end)
  local w = {}
  for j = 1, n do w[j] = weights and weights[j] or 1 end
  local function unit(k)
    local b = {}
    for i = 1, #rows do b[i] = (i == k) and 1 or 0 end
    return M.minNorm(B, b, w)
  end
  local u = unit(1)
  local ax = rollRow and unit(rollRow) or nil
  local az = pitchRow and unit(pitchRow) or nil
  if not u or (rollRow and not ax) or (pitchRow and not az) then return nil, "rotor layout cannot produce the lift and torques it should (degenerate)" end
  return { n = n, rotors = rotors, B = B, u = u, ax = ax, az = az, w = w, rollTilt = rollTilt, pitchTilt = pitchTilt }
end

-- Thrusts for total vertical force c, torques tx, tz (only the thrust-driven ones are used). Returns T (array), info
function A.thrust(self, c, tx, tz)
  local n = self.n
  local a, T = {}, {}
  for j = 1, n do a[j] = (self.ax and tx * self.ax[j] or 0) + (self.az and tz * self.az[j] or 0) end
  local function interval(k)
    local lo, hi = -math.huge, math.huge
    for j = 1, n do
      local r = self.rotors[j]
      local base = k * a[j]
      local uj = self.u[j]
      local tl, th = (r.tmin or 0) - base, (r.tmax or math.huge) - base
      if uj > 1e-9 then
        lo, hi = math.max(lo, tl / uj), math.min(hi, th / uj)
      elseif uj < -1e-9 then
        lo, hi = math.max(lo, th / uj), math.min(hi, tl / uj)
      elseif base < (r.tmin or 0) - 1e-9 or base > (r.tmax or math.huge) + 1e-9 then
        return 1, 0           -- infeasible regardless of c
      end
    end
    return lo, hi
  end
  local k = 1
  local lo, hi = interval(1)
  if lo > hi then
    -- attitude demand alone does not fit: shrink it until some collective works
    local klo, khi = 0, 1
    for _ = 1, 30 do
      local km = (klo + khi) / 2
      local l, h = interval(km)
      if l <= h then klo = km else khi = km end
    end
    k = klo
    lo, hi = interval(k)
    if lo > hi then lo, hi = c, c end
  end
  local cc = M.clamp(c, lo, hi)
  for j = 1, n do T[j] = cc * self.u[j] + k * a[j] end
  return T, { c = cc, attScale = k, saturated = (k < 1) or (cc ~= c) }
end

-- Tilts (tx_j, tz_j) for lateral force (Fx, Fz), yaw torque tauY and, for tilt-driven axes, roll torque tauRoll / pitch torque tauPitch, given the
-- rotor thrusts T. coneTan = tan(max tilt). Returns tilts = {{tx,tz},...}, info{scale, tauX, tauZ (torques the tilts add), Fx, Fz, tauY produced}
function A.tilts(self, T, Fx, Fz, tauY, coneTan, tauRoll, tauPitch)
  local n = self.n
  local R = self.rotors
  local rows, rhs = {}, {}
  local function add(fn, v)
    local row = {}
    for j = 1, 2 * n do row[j] = 0 end
    for j = 1, n do
      local ctx, ctz = fn(R[j], T[j])
      row[j], row[n + j] = ctx, ctz
    end
    rows[#rows + 1] = row
    rhs[#rhs + 1] = v
  end
  if not self.pitchTilt then add(function(r, t) return t, 0 end, Fx) end                       -- lateral force along x
  if not self.rollTilt then add(function(r, t) return 0, t end, Fz) end                        -- lateral force along z
  add(function(r, t) return r.z * t, -r.x * t end, tauY)                                       -- yaw torque
  if self.rollTilt then add(function(r, t) return 0, r.y * t end, tauRoll or 0) end            -- roll torque from the z tilts
  if self.pitchTilt then add(function(r, t) return -r.y * t, 0 end, tauPitch or 0) end         -- pitch torque from the x tilts
  local sol = M.minNorm(rows, rhs)
  if not sol then return nil, "tilt allocation singular" end
  local worst = 0
  for j = 1, n do worst = math.max(worst, math.sqrt(sol[j] ^ 2 + sol[n + j] ^ 2)) end
  local s = 1
  if worst > coneTan then s = coneTan / worst end
  local out, tauX, tauZ, pFx, pFz, pTy = {}, 0, 0, 0, 0, 0
  for j = 1, n do
    local tx, tz = sol[j] * s, sol[n + j] * s
    out[j] = { tx, tz }
    tauX = tauX + R[j].y * T[j] * tz
    tauZ = tauZ - R[j].y * T[j] * tx
    pFx = pFx + T[j] * tx
    pFz = pFz + T[j] * tz
    pTy = pTy + T[j] * (R[j].z * tx - R[j].x * tz)
  end
  return out, { scale = s, tauX = tauX, tauZ = tauZ, Fx = pFx, Fz = pFz, tauY = pTy }
end

return A
]],
  ["ap/lib/ctl.lua"] = [[
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
]],
  ["ap/lib/diag.lua"] = [[
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
]],
  ["ap/lib/fly.lua"] = [[
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
]],
  ["ap/lib/hal.lua"] = [[
-- ap/lib/hal.lua : hardware abstraction. The SAME interface is served by the real ship (peripherals + sublevel API) and by
-- the simulator, so the wizard and the flight program are tested in `ap selftest` without moving anything.
--
--   hal.n                      number of rotors
--   hal.sense()                -> {q={w,x,y,z} body->world, p={x,y,z}, v={x,y,z}, w={x,y,z} WORLD angular velocity rad/s}
--   hal.cycle(rpm, targets)    actuate AND read the next sample in one batch (what the flight loop uses)
--   hal.actuate(rpm, targets)  one batched write: motor RPM per rotor, bearing target {x,y,z} (ship-LOCAL) per rotor (or nil)
--   hal.probe(j)               -> {speed, thrust, vec={x,y,z}, handed}   (bearing readings)
--   hal.setLevel(j, level)     transmission shift level (no-op without transmissions)
--   hal.setHanded(j, name)     thrust handedness
--   hal.now()                  seconds (sim: simulated time)   hal.wait(s)   sleep (sim: advances the sim with the last command)
--   hal.stop()                 zero the rotors and restore the original motor settings
local M = require("m")
local H = {}

local unpack = table.unpack or unpack

---------------------------------------------------------------------------------------------------------------------
-- real ship
---------------------------------------------------------------------------------------------------------------------
-- af.rotors[j] = {motor=, bearing=, trans=}
function H.real(af)
  local R = af.rotors
  local self = { n = #R, real = true, orig = {}, tilt = true }
  for j, r in ipairs(R) do
    local ok, v = pcall(peripheral.call, r.motor, "getGeneratedSpeed")
    self.orig[j] = ok and v or 0
  end
  local lastPos
  local function readJobs(store)
    return {
      function()
        if aero and aero.getAirPressure and vector and lastPos then
          local ok, p = pcall(aero.getAirPressure, vector.new(lastPos.x, lastPos.y, lastPos.z))
          if ok and type(p) == "number" then store.press = p end
        end
      end,
      function() store.pose = sublevel.getLogicalPose() end,
      function() store.vel = sublevel.getLinearVelocity() end,
      function() store.ang = sublevel.getAngularVelocity() end,
    }
  end
  local function unpackSample(st)
    local o = st.pose.orientation
    local q = M.qnorm({ w = o.a, x = o.v.x, y = o.v.y, z = o.v.z })
    -- MEASURED 2026-11-16 (tricopter flight): sublevel.getAngularVelocity is in the SHIP's BODY frame (correlation 1.00 with the
    -- quaternion-derived rate), not the world frame. The HAL contract is a world vector, so rotate it here.
    lastPos = st.pose.position
    return {
      q = q, pressure = st.press,
      p = { x = st.pose.position.x, y = st.pose.position.y, z = st.pose.position.z },
      v = { x = st.vel.x, y = st.vel.y, z = st.vel.z },
      w = M.qrot(q, { x = st.ang.x, y = st.ang.y, z = st.ang.z }),
    }
  end
  local function writeJobs(rpm, targets)
    local jobs = {}
    for j, r in ipairs(R) do
      jobs[#jobs + 1] = function() peripheral.call(r.motor, "setGeneratedSpeed", rpm[j]) end
      if targets and targets[j] then
        local tg = targets[j]
        jobs[#jobs + 1] = function() peripheral.call(r.bearing, "setManualTarget", { tg.x, tg.y, tg.z }) end
      end
    end
    return jobs
  end
  function self.sense()
    local st = {}
    parallel.waitForAll(unpack(readJobs(st)))
    return unpackSample(st)
  end
  -- one parallel batch: this step's writes AND the next sample's reads share a single server tick (20 Hz loop)
  function self.cycle(rpm, targets)
    local st = {}
    local jobs = writeJobs(rpm, targets)
    for _, f in ipairs(readJobs(st)) do jobs[#jobs + 1] = f end
    parallel.waitForAll(unpack(jobs))
    return unpackSample(st)
  end
  function self.actuate(rpm, targets)
    parallel.waitForAll(unpack(writeJobs(rpm, targets)))
  end
  function self.probe(j)
    local b = R[j].bearing
    local function c(m) local ok, v = pcall(peripheral.call, b, m) if ok then return v end end
    local tv = c("getThrustVector")
    return {
      speed = c("getSpeed") or 0, thrust = c("getThrust") or 0,
      vec = tv and { x = tv[1], y = tv[2], z = tv[3] } or nil,
      handed = c("getThrustHandedness"),
    }
  end
  function self.setLevel(j, level)
    if R[j].trans then peripheral.call(R[j].trans, "setShiftLevel", level) end
  end
  function self.setHanded(j, name) peripheral.call(R[j].bearing, "setThrustHandedness", name) end
  -- air pressure at an altitude above the ship (nil when the API is missing): decides the cruise height of a transit
  function self.pressureAt(x, y, z)
    if not (aero and aero.getAirPressure and vector) then return nil end
    local ok, p = pcall(aero.getAirPressure, vector.new(x, y, z))
    if ok and type(p) == "number" then return p end
  end
  function self.now() return os.epoch("utc") / 1000 end
  function self.wait(s) sleep(s) end
  function self.stop()
    local jobs = {}
    for j, r in ipairs(R) do
      jobs[#jobs + 1] = function() peripheral.call(r.motor, "setGeneratedSpeed", 0) end
      if r.trans then jobs[#jobs + 1] = function() peripheral.call(r.trans, "setShiftLevel", 0) end end
      jobs[#jobs + 1] = function() pcall(peripheral.call, r.bearing, "clearManualTarget") end
    end
    pcall(parallel.waitForAll, unpack(jobs))
    local back = {}
    for j, r in ipairs(R) do
      back[#back + 1] = function() peripheral.call(r.motor, "setGeneratedSpeed", self.orig[j] or 0) end
    end
    pcall(parallel.waitForAll, unpack(back))
  end
  return self
end

---------------------------------------------------------------------------------------------------------------------
-- simulated ship. truth = {kT = {per bearing index}, trans = {bool per bearing index}, motorDrives = {motorName = bearingName},
-- transDrives = {transName = bearingName}, goodHanded = {per bearing index}, liftSign = -1 (getThrust sign when lifting, as on the
-- real bearings with left_handed), ground = true}. `af` supplies the (possibly wrong) names the program believes in.
---------------------------------------------------------------------------------------------------------------------
local Sim = require("sim")

function H.sim(simCfg, truth, af)
  local sim = Sim.new(simCfg)
  local n = #simCfg.rotors
  local self = { n = n, real = false, sim = sim, truth = truth, rpm = {}, handed = {}, level = {}, cmdT = {}, cmdTilt = {}, dt = 0.05 }
  local idx = {}
  for j = 1, n do
    idx[af.rotors[j].bearing] = j
    self.rpm[j] = 0
    self.handed[j] = (truth.startHanded and truth.startHanded[j]) or "left_handed"
    self.level[j] = (truth.trans and truth.trans[j]) and 0 or nil     -- nil: direct drive, no transmission
    self.cmdT[j] = 0
    self.cmdTilt[j] = { 0, 0 }
  end
  local function bearingRpm(j)
    local lv = self.level[j]
    if lv then return self.rpm[j] * lv / 15 end
    return self.rpm[j]
  end
  local function lifts(j)
    return self.handed[j] == ((truth.goodHanded and truth.goodHanded[j]) or "left_handed")
  end
  local function thrustOf(j)
    local t = truth.kT[j] * bearingRpm(j)
    return lifts(j) and t or -t
  end
  function self.sense()
    local q = M.qnorm(sim.q)
    return {
      q = { w = q.w, x = q.x, y = q.y, z = q.z },
      pressure = simCfg.pressure and simCfg.pressure(sim.p.y) or nil,
      p = { x = sim.p.x, y = sim.p.y, z = sim.p.z },
      v = { x = sim.v.x, y = sim.v.y, z = sim.v.z },
      w = M.qrot(q, sim.w),
    }
  end
  local function advance()
    -- ground: the ship cannot sink below its start height
    Sim.step(sim, self.cmdT, self.cmdTilt, self.dt)
    if sim.p.y <= (simCfg.y0 or 20) and truth.ground ~= false then
      sim.p.y = simCfg.y0 or 20
      if sim.v.y < 0 then sim.v.y = 0 end
      sim.v.x, sim.v.z = sim.v.x * 0.5, sim.v.z * 0.5
      -- standing on its wheels: the ship cannot tilt, only turn
      local psi = M.attitude(sim.q).psi / M.DEG
      sim.q = { w = math.cos(psi / 2), x = 0, y = math.sin(psi / 2), z = 0 }
      sim.w = { x = 0, y = sim.w.y * 0.5, z = 0 }
    end
  end
  function self.actuate(rpm, targets)
    for j = 1, n do self.rpm[j] = 0 end
    for j = 1, n do
      local b = truth.motorDrives[af.rotors[j].motor]
      local i = b and idx[b]
      if i then self.rpm[i] = M.clamp(rpm[j], 0, 256) end
    end
    for i = 1, n do
      self.cmdT[i] = math.max(0, thrustOf(i))
      local tg = targets and targets[i]
      if tg and tg.y ~= 0 then self.cmdTilt[i] = { tg.x / tg.y, tg.z / tg.y } else self.cmdTilt[i] = { 0, 0 } end
    end
    advance()
  end
  function self.probe(j)
    local sp = bearingRpm(j)
    local d = lifts(j) and 1 or -1
    -- the real bearings report NEGATIVE getThrust for lifting thrust (left_handed); the thrust vector is the direction of the force
    return { speed = sp, thrust = d * (truth.liftSign or -1) * truth.kT[j] * sp, vec = { x = 0, y = d, z = 0 }, handed = self.handed[j] }
  end
  function self.setLevel(j, level)
    local b = af.rotors[j].trans and truth.transDrives[af.rotors[j].trans]
    local i = b and idx[b]
    if i and self.level[i] then self.level[i] = level end
  end
  function self.setHanded(j, name) self.handed[j] = name end
  function self.cycle(rpm, targets) self.actuate(rpm, targets) return self.sense() end
  function self.pressureAt(x, y, z) return simCfg.pressure and simCfg.pressure(y) or nil end
  function self.now() return sim.t end
  function self.wait(s) end
  function self.stop()
    for j = 1, n do self.rpm[j] = 0; self.cmdT[j] = 0 end
    advance()
  end
  return self
end

return H
]],
  ["ap/lib/hw.lua"] = [=[
-- ap/lib/hw.lua : hardware discovery. Finds the rotors (motor + optional transmission + gyroscopic bearing) of the ship this
-- computer is on, and works out WHERE each rotor is.
--
-- MEASURED 2026-11-16 on two ships: the ids returned by getSelfId / getNetworkId / getSourceId are the block position packed
-- like Minecraft's BlockPos.asLong (x: top 26 bits, z: next 26 bits, y: low 12 bits) in the ship's plot grid, which is the
-- same frame as sublevel.getCenterOfMass() (plot coordinates, ship body axes). So rotor position relative to the centre of
-- mass is  blockCenter - CoM  with no flight test. A motor and its bearing sit in the same x/z column.
local H = {}

local function hexNum(s) return tonumber(s, 16) end

-- "4e20ff9388408083" -> x, y, z (plot block coordinates), or nil
function H.decodeId(id)
  if type(id) ~= "string" or #id ~= 16 or not id:match("^%x+$") then return nil end
  local hi, lo = hexNum(id:sub(1, 8)), hexNum(id:sub(9, 16))
  local x = math.floor(hi / 64)
  local z = (hi % 64) * 1048576 + math.floor(lo / 4096)
  local y = lo % 4096
  if x >= 33554432 then x = x - 67108864 end
  if z >= 33554432 then z = z - 67108864 end
  if y >= 2048 then y = y - 4096 end
  return x, y, z
end

local function safe(name, method, ...)
  local ok, v = pcall(peripheral.call, name, method, ...)
  if ok then return v end
  return nil
end

local function namesByType(t)
  local out = {}
  for _, n in ipairs(peripheral.getNames()) do
    local ok, ty = pcall(peripheral.getType, n)
    if ok and ty then
      if type(ty) == "string" and (ty == t or ty:find(t, 1, true)) then out[#out + 1] = n end
    end
  end
  table.sort(out)
  return out
end

-- zone word from the peripheral name: ship_gyroscopic_propeller_bearing_fore_port_0 -> "fore_port"
local function zoneOf(bearingName)
  local z = bearingName:match("propeller_bearing_(.+)_%d+$")
  return z or bearingName
end

-- Returns {rotors = {...}, mass, com = {x,y,z}, notes = {...}, warnings = {...}}
-- Each rotor: {name, bearing, motor, trans, zone, pos = {x,y,z} (body frame, m, relative to CoM), blockPos, handed, ...}
function H.scan()
  local info = { rotors = {}, notes = {}, warnings = {} }
  local function warn(s) info.warnings[#info.warnings + 1] = s end
  if not (sublevel and sublevel.getMass) then warn("no sublevel API: this computer is not on a Sable ship") return info end
  local okc, com = pcall(sublevel.getCenterOfMass)
  if okc and com then info.com = { x = com.x, y = com.y, z = com.z } else warn("cannot read the centre of mass") end
  local okm, mass = pcall(sublevel.getMass)
  info.mass = okm and mass or nil
  local oki, tensor = pcall(sublevel.getInertiaTensor)
  if oki and tensor and tensor[1] then info.inertia = { x = tensor[1][1], y = tensor[2][2], z = tensor[3][3] } end
  info.g = 11
  if aero and aero.getGravity then
    local okg, gv = pcall(aero.getGravity)
    if okg and gv and gv.y and gv.y ~= 0 then info.g = math.abs(gv.y) end
  end

  local bearings = namesByType("propeller_bearing")
  local motors = namesByType("CreativeMotor")
  local trans = namesByType("redstone_transmission")
  info.nMotors, info.nBearings, info.nTrans = #motors, #bearings, #trans
  if #bearings == 0 then warn("no gyroscopic propeller bearings found") end

  -- motor lookup tables: by network id (direct drive) and by x/z column
  local motorByNet, motorByCol = {}, {}
  for _, m in ipairs(motors) do
    local nid = safe(m, "getNetworkId")
    if nid then motorByNet[nid] = m end
    local sx, sy, sz = H.decodeId(safe(m, "getSelfId"))
    if sx then motorByCol[sx .. "," .. sz] = m end
  end

  local usedTrans = {}
  for _, b in ipairs(bearings) do
    local r = { bearing = b, zone = zoneOf(b), name = zoneOf(b) }
    r.assembled = safe(b, "isAssembled")
    r.handed = safe(b, "getThrustHandedness")
    r.axis = safe(b, "getAxis")
    local sid = safe(b, "getSelfId")
    local bx, by, bz = H.decodeId(sid)
    if bx then
      r.blockPos = { x = bx, y = by, z = bz }
      if info.com then
        r.pos = { x = bx + 0.5 - info.com.x, y = by + 0.5 - info.com.y, z = bz + 0.5 - info.com.z }
      end
    else
      warn(b .. ": position id not decodable")
    end
    -- motor: network id first (direct drive), else same x/z column
    local src = safe(b, "getSourceId")
    if src and motorByNet[src] then r.motor = motorByNet[src]; r.link = "network"
    elseif bx and motorByCol[bx .. "," .. bz] then r.motor = motorByCol[bx .. "," .. bz]; r.link = "column" end
    if not r.motor then warn(b .. ": no motor found") end
    info.rotors[#info.rotors + 1] = r
  end
  table.sort(info.rotors, function(a, c) return a.bearing < c.bearing end)

  -- transmissions carry no ids: pair by motor index order when the counts match (verified by the wizard's spin test)
  if #trans > 0 then
    if #trans == #motors then
      local idx = {}
      for i, m in ipairs(motors) do idx[m] = i end
      for _, r in ipairs(info.rotors) do
        if r.motor and idx[r.motor] then r.trans = trans[idx[r.motor]]; r.transGuess = true end
      end
      info.notes[#info.notes + 1] = "transmissions paired to motors by index (unverified until the wizard spin test)"
    else
      warn(#trans .. " transmissions but " .. #motors .. " motors: pairing needs the wizard")
    end
  end
  return info
end

-- sanity check of a scan result; returns a list of problems (empty = fine)
function H.check(info)
  local bad = {}
  for _, w in ipairs(info.warnings) do bad[#bad + 1] = w end
  if #info.rotors < 2 then bad[#bad + 1] = "need at least 2 rotors, found " .. #info.rotors end
  local ymin, ymax
  for _, r in ipairs(info.rotors) do
    if not r.assembled then bad[#bad + 1] = r.bearing .. " is not assembled" end
    if r.axis ~= "up" then bad[#bad + 1] = r.bearing .. " axis is " .. tostring(r.axis) .. " (only 'up' rotors are supported so far)" end
    if not r.pos then bad[#bad + 1] = r.bearing .. " has no position" end
  end
  return bad
end

return H
]=],
  ["ap/lib/m.lua"] = [[
-- ap/lib/m.lua : small pure math helpers (vectors, quaternions, dense linear algebra). No peripherals.
local M = {}

M.DEG = 180 / math.pi

function M.clamp(v, lo, hi) if v < lo then return lo elseif v > hi then return hi end return v end
function M.wrap180(a) return (a + 180) % 360 - 180 end

-- quaternions {w,x,y,z}, body->world ----------------------------------------------------------
function M.qnorm(q)
  local n = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)
  if n == 0 then return { w = 1, x = 0, y = 0, z = 0 } end
  return { w = q.w / n, x = q.x / n, y = q.y / n, z = q.z / n }
end
function M.qconj(q) return { w = q.w, x = -q.x, y = -q.y, z = -q.z } end
function M.qrot(q, v)
  local tx = 2 * (q.y * v.z - q.z * v.y)
  local ty = 2 * (q.z * v.x - q.x * v.z)
  local tz = 2 * (q.x * v.y - q.y * v.x)
  return {
    x = v.x + q.w * tx + (q.y * tz - q.z * ty),
    y = v.y + q.w * ty + (q.z * tx - q.x * tz),
    z = v.z + q.w * tz + (q.x * ty - q.y * tx),
  }
end
function M.qmul(a, b)
  return {
    w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  }
end
-- integrate body angular velocity w (body frame, rad/s) over dt
function M.qintegrate(q, w, dt)
  local ang = math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z) * dt
  if ang < 1e-12 then return q end
  local s = math.sin(ang / 2) / (ang / dt)
  return M.qnorm(M.qmul(q, { w = math.cos(ang / 2), x = w.x * s, y = w.y * s, z = w.z * s }))
end

-- rotation that takes orientation qa to qb, as a rotation VECTOR in the body frame of qa (rad): angle * axis
function M.qdelta(qa, qb)
  local r = M.qmul(M.qconj(M.qnorm(qa)), M.qnorm(qb))
  if r.w < 0 then r = { w = -r.w, x = -r.x, y = -r.y, z = -r.z } end
  local s = math.sqrt(r.x * r.x + r.y * r.y + r.z * r.z)
  if s < 1e-12 then return { x = 0, y = 0, z = 0 } end
  local ang = 2 * math.atan2(s, r.w)
  return { x = r.x / s * ang, y = r.y / s * ang, z = r.z / s * ang }
end

-- Attitude from a body->world quaternion. thX/thZ: tilt angles (deg) of body up about the body X/Z axes,
-- psi: heading (deg, rotation about world +Y, body +X projected on the ground plane), tilt total.
function M.attitude(q)
  q = M.qnorm(q)
  local up = M.qrot(M.qconj(q), { x = 0, y = 1, z = 0 })
  local upW = M.qrot(q, { x = 0, y = 1, z = 0 })
  local fw = M.qrot(q, { x = 1, y = 0, z = 0 })
  return {
    thX = math.atan2(-up.x, up.y) * M.DEG,
    thZ = math.atan2(-up.z, up.y) * M.DEG,
    tilt = math.acos(M.clamp(upW.y, -1, 1)) * M.DEG,
    cosTilt = upW.y,
    psi = -math.atan2(fw.z, fw.x) * M.DEG,
  }
end

-- dense linear algebra on arrays of rows --------------------------------------------------------
function M.mat(r, c, f)
  local A = {}
  for i = 1, r do A[i] = {} for j = 1, c do A[i][j] = f and f(i, j) or 0 end end
  return A
end
function M.transpose(A)
  local r, c = #A, #A[1]
  return M.mat(c, r, function(i, j) return A[j][i] end)
end
function M.mul(A, B)
  local r, k, c = #A, #B, #B[1]
  return M.mat(r, c, function(i, j)
    local s = 0
    for t = 1, k do s = s + A[i][t] * B[t][j] end
    return s
  end)
end
function M.mulv(A, v)
  local out = {}
  for i = 1, #A do
    local s = 0
    for j = 1, #v do s = s + A[i][j] * v[j] end
    out[i] = s
  end
  return out
end

-- Solve A x = b (square) by Gaussian elimination with partial pivoting. Returns nil if singular.
function M.solve(A, b)
  local n = #A
  local a = {}
  for i = 1, n do
    a[i] = {}
    for j = 1, n do a[i][j] = A[i][j] end
    a[i][n + 1] = b[i]
  end
  for col = 1, n do
    local piv, best = col, math.abs(a[col][col])
    for r = col + 1, n do
      if math.abs(a[r][col]) > best then piv, best = r, math.abs(a[r][col]) end
    end
    if best < 1e-12 then return nil end
    a[col], a[piv] = a[piv], a[col]
    for r = col + 1, n do
      local f = a[r][col] / a[col][col]
      if f ~= 0 then for c = col, n + 1 do a[r][c] = a[r][c] - f * a[col][c] end end
    end
  end
  local x = {}
  for i = n, 1, -1 do
    local s = a[i][n + 1]
    for j = i + 1, n do s = s - a[i][j] * x[j] end
    x[i] = s / a[i][i]
  end
  return x
end

-- Minimum-norm solution of  A x = b  with a per-unknown weight w (cost = sum w_j x_j^2), A is m x n with m <= n.
-- x = W^-1 A^T (A W^-1 A^T)^-1 b.  Returns nil if A is rank deficient.
function M.minNorm(A, b, w)
  local m, n = #A, #A[1]
  local G = M.mat(m, m)
  for i = 1, m do
    for j = 1, m do
      local s = 0
      for k = 1, n do s = s + A[i][k] * A[j][k] / (w and w[k] or 1) end
      G[i][j] = s
    end
  end
  local y = M.solve(G, b)
  if not y then return nil end
  local x = {}
  for k = 1, n do
    local s = 0
    for i = 1, m do s = s + A[i][k] * y[i] end
    x[k] = s / (w and w[k] or 1)
  end
  return x
end

return M
]],
  ["ap/lib/nav.lua"] = [[
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
]],
  ["ap/lib/sim.lua"] = [[
-- ap/lib/sim.lua : tiny rigid-body multirotor simulator used by `ap selftest` (pure Lua, runs inside ComputerCraft).
-- Rotors produce thrust along their tilted direction with a pure command delay; no reaction torque (the gyroscopic
-- bearings do not produce any on the real ships). Used to prove the allocator and the controller before anything flies.
local M = require("m")
local S = {}

-- cfg: {mass, inertia={x,y,z}, g, rotors={{x,y,z,kT,tmax}...}, delay (s), cone (deg), drag, dt}
function S.new(cfg)
  local s = {
    cfg = cfg,
    m = cfg.mass, I = cfg.inertia, g = cfg.g or 11,
    p = { x = 0, y = cfg.y0 or 20, z = 0 },
    v = { x = 0, y = 0, z = 0 },
    q = { w = 1, x = 0, y = 0, z = 0 },
    w = { x = 0, y = 0, z = 0 },
    t = 0,
    T = {}, tilt = {},              -- delivered thrust / tilt
    queue = {},
    thrustSign = 1,
  }
  for j, r in ipairs(cfg.rotors) do s.T[j] = 0; s.tilt[j] = { 0, 0 } end
  return s
end

local function cross(a, b)
  return { x = a.y * b.z - a.z * b.y, y = a.z * b.x - a.x * b.z, z = a.x * b.y - a.y * b.x }
end

-- advance one control period dt with commands cmdT[j] (thrust) and cmdTilt[j] = {tx,tz}; sub-stepped internally
function S.step(s, cmdT, cmdTilt, dt)
  local cfg = s.cfg
  local delay = cfg.delay or 0.2
  -- copy the commands: callers reuse their arrays every step, so queued entries must not alias them (the delay would vanish)
  local qT, qTilt = {}, {}
  for j = 1, #cfg.rotors do
    qT[j] = cmdT[j]
    local tl = cmdTilt and cmdTilt[j] or { 0, 0 }
    qTilt[j] = { tl[1], tl[2] }
  end
  s.queue[#s.queue + 1] = { T = qT, tilt = qTilt, t = s.t }
  -- release the newest command that is at least `delay` old
  local use
  while #s.queue > 0 and s.queue[1].t <= s.t - delay + 1e-9 do use = table.remove(s.queue, 1) end
  if use then
    for j = 1, #cfg.rotors do
      local r = cfg.rotors[j]
      s.T[j] = M.clamp(use.T[j] * (r.gainErr or 1), r.tmin or 0, r.tmax or math.huge)
      local tl = use.tilt and use.tilt[j] or { 0, 0 }
      local cone = math.tan((cfg.cone or 11.5) / M.DEG)
      local mag = math.sqrt(tl[1] ^ 2 + tl[2] ^ 2)
      local k = (mag > cone) and cone / mag or 1
      local ls = cfg.latScale     -- optional weak sides: tilt effectiveness per direction {xp, xn, zp, zn}
      local fx = (ls and ((tl[1] >= 0) and (ls.xp or 1) or (ls.xn or 1))) or 1
      local fz = (ls and ((tl[2] >= 0) and (ls.zp or 1) or (ls.zn or 1))) or 1
      s.tilt[j] = { tl[1] * k * fx, tl[2] * k * fz }
    end
  end
  local sub = math.max(1, math.floor(dt / 0.01 + 0.5))
  local h = dt / sub
  for _ = 1, sub do
    local F = { x = 0, y = 0, z = 0 }
    local tau = { x = 0, y = 0, z = 0 }
    -- thrust drops with air flowing along the propeller axis (v . axis): climbing or leaning INTO the motion loses lift, leaning back gains
    local axW = M.qrot(s.q, { x = 0, y = 1, z = 0 })
    local axial = s.v.x * axW.x + s.v.y * axW.y + s.v.z * axW.z
    local infl = M.clamp(1 - (cfg.inflowLoss or 0) * axial, 0.2, 1.4) * (cfg.pressure and cfg.pressure(s.p.y) or 1)
    for j, r in ipairs(cfg.rotors) do
      local tl = s.tilt[j]
      local n = math.sqrt(tl[1] ^ 2 + 1 + tl[2] ^ 2)
      local f = { x = s.T[j] * infl * tl[1] / n, y = s.T[j] * infl / n, z = s.T[j] * infl * tl[2] / n }
      F.x, F.y, F.z = F.x + f.x, F.y + f.y, F.z + f.z
      local c = cross({ x = r.x, y = r.y, z = r.z }, f)
      tau.x, tau.y, tau.z = tau.x + c.x, tau.y + c.y, tau.z + c.z
    end
    local ts = cfg.torqueScale      -- optional torque authority error {x, y, z}
    if ts then tau.x, tau.y, tau.z = tau.x * (ts.x or 1), tau.y * (ts.y or 1), tau.z * (ts.z or 1) end
    local Fw = M.qrot(s.q, F)
    local drag = cfg.drag or 0
    local sp = math.sqrt(s.v.x ^ 2 + s.v.y ^ 2 + s.v.z ^ 2)
    local ax = Fw.x / s.m - drag * sp * s.v.x
    local ay = Fw.y / s.m - s.g - drag * sp * s.v.y
    local az = Fw.z / s.m - drag * sp * s.v.z
    s.v.x, s.v.y, s.v.z = s.v.x + ax * h, s.v.y + ay * h, s.v.z + az * h
    s.p.x, s.p.y, s.p.z = s.p.x + s.v.x * h, s.p.y + s.v.y * h, s.p.z + s.v.z * h
    local Iw = { x = s.I.x * s.w.x, y = s.I.y * s.w.y, z = s.I.z * s.w.z }
    local gyro = cross(s.w, Iw)
    s.w.x = s.w.x + (tau.x - gyro.x) / s.I.x * h
    s.w.y = s.w.y + (tau.y - gyro.y) / s.I.y * h
    s.w.z = s.w.z + (tau.z - gyro.z) / s.I.z * h
    s.q = M.qintegrate(s.q, s.w, h)
    s.t = s.t + h
  end
end

-- Standard test layouts. arms in metres relative to the CoM. mass 80, g 11 like the real ships.
function S.layout(name)
  local function ring(n, rad, phase)
    local out = {}
    for j = 1, n do
      local a = (phase or 0) + 2 * math.pi * (j - 1) / n
      out[j] = { x = rad * math.cos(a), y = 1.5, z = rad * math.sin(a) }
    end
    return out
  end
  if name == "tri" then
    -- the real tricopter: aft rotor behind, two forward
    return { { x = 0, y = 1.3, z = -6.5 }, { x = 4, y = 1.3, z = 2.5 }, { x = -4, y = 1.3, z = 2.5 } }, 36.5
  elseif name == "tandem" then
    -- two-rotor tandem helicopter (the real one: mass 537, rotors 17 m apart on the x axis, 3-5 m above the centre of mass)
    return { { x = -11.1, y = 5.0, z = 0 }, { x = 5.9, y = 3.0, z = 0 } }, 537
  elseif name == "quad+" then
    return { { x = -9.7, y = 1.9, z = 0 }, { x = 8.3, y = 1.9, z = 0 }, { x = -0.7, y = 1.9, z = 10 }, { x = -0.7, y = 1.9, z = -10 } }, 80.5
  elseif name == "quadx" then return ring(4, 8, math.pi / 4), 60
  elseif name == "hex" then return ring(6, 9, 0), 90
  elseif name == "octo" then return ring(8, 10, 0.2), 110
  elseif name == "offset" then
    -- asymmetric five-rotor craft with the heavy side forward
    return { { x = 6, y = 1, z = 0 }, { x = -3, y = 1, z = 7 }, { x = -3, y = 1, z = -7 }, { x = -9, y = 1, z = 3 }, { x = -9, y = 1, z = -3 } }, 70
  end
end

-- build a sim config from a layout: hover thrust per rotor ~ weight / n, tmax = 2.4 x that
function S.config(name, over)
  local arms, mass = S.layout(name)
  local g = 11
  local rotors = {}
  local share = mass * g / #arms
  local ixx, izz, iyy = 0, 0, 0
  for j, a in ipairs(arms) do
    -- the offset craft needs one rotor to carry ~45 % of the weight, so give it more headroom
    rotors[j] = { x = a.x, y = a.y, z = a.z, tmax = share * (name == "offset" and 3.6 or 2.4), tmin = 0 }
    ixx = ixx + (a.y ^ 2 + a.z ^ 2) * mass / #arms
    izz = izz + (a.x ^ 2 + a.y ^ 2) * mass / #arms
    iyy = iyy + (a.x ^ 2 + a.z ^ 2) * mass / #arms
  end
  local cfg = { mass = mass, g = g, rotors = rotors, inertia = { x = ixx * 1.4, y = iyy * 1.4, z = izz * 1.4 }, delay = 0.2, cone = 11.5, drag = 0.0005 }
  if name == "tandem" then cfg.inertia = { x = 5547, y = 35709, z = 36127 } end      -- measured on the real tandem
  for k, v in pairs(over or {}) do cfg[k] = v end
  return cfg
end

return S
]],
  ["ap/lib/tune.lua"] = [[
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
]],
  ["ap/lib/wizard.lua"] = [=[
-- ap/lib/wizard.lua : ground-only setup wizard. Works against any HAL (the real ship or the simulator).
--   1. checks the scan (positions, assembled, up-facing)  2. verifies which motor / transmission drives which bearing
--   3. makes sure every bearing pushes UP (handedness)    4. measures thrust per bearing RPM for every rotor
--   5. chooses transmission levels and thrust limits       6. writes /ap_data/airframe.json
-- Safety: only low speeds are used (total lift stays under ~25 % of the weight), so the ship cannot leave the ground.
local H = require("hw")
local A = require("airframe")
local Al = require("alloc")
local W = {}

local SPIN = 16          -- motor RPM for the identification spins
local SPIN_LEVEL = 5     -- transmission level used while spinning

-- airframe skeleton from a hardware scan
function W.skeleton(info)
  local af = { version = 1, mass = info.mass, g = info.g or 11, inertia = info.inertia, motorCap = 250, coneDeg = 11.5, rotors = {} }
  for j, r in ipairs(info.rotors) do
    af.rotors[j] = {
      zone = r.zone, bearing = r.bearing, motor = r.motor, trans = r.trans, hasTrans = r.trans ~= nil,
      x = r.pos.x, y = r.pos.y, z = r.pos.z, handed = r.handed,
    }
  end
  return af
end

local function zeros(n) local t = {} for j = 1, n do t[j] = 0 end return t end

local function settle(hal) hal.wait(1.5) end

-- which bearings are turning right now (speed above 1)?
local function turning(hal, n)
  local out = {}
  for j = 1, n do
    local p = hal.probe(j)
    if math.abs(p.speed or 0) > 1 then out[#out + 1] = j end
  end
  return out
end

function W.run(hal, af, io)
  local say, ask = io.say, io.ask
  local n = #af.rotors
  local report = { fixes = {} }
  local function fail(msg) say("WIZARD STOPPED: " .. msg) pcall(hal.stop) return nil, msg end

  say(string.format("ship: %d rotors, mass %.1f, g %.1f", n, af.mass or 0, af.g or 0))
  if not af.inertia then return fail("cannot read the inertia tensor") end
  for _, r in ipairs(af.rotors) do
    say(string.format("  %-16s arm x %+.1f y %+.1f z %+.1f  motor %s%s", r.zone, r.x, r.y, r.z, tostring(r.motor), r.trans and ("  trans " .. r.trans) or ""))
    if not r.motor then return fail(r.zone .. " has no motor") end
  end
  local al, aerr = Al.new(af.rotors)
  if not al then return fail(aerr) end
  local minShare = 1
  for j = 1, n do minShare = math.min(minShare, al.u[j]) end
  if minShare < 0.02 then return fail("this rotor layout cannot hover with upward thrust only (a rotor would have to push down)") end

  -- the front of the ship: default = the fore side of the avionics controller (or the front chosen in an earlier setup)
  do
    local axis, why = A.defaultFront(af.rotors)
    local src = "fore"
    if io.prevFront and io.prevFront.axis then axis, why, src = io.prevFront.axis, "the front chosen at the last setup", io.prevFront.src or "manual" end
    say(string.format("front of the ship: %s (%s)", axis, why))
    if io.line then
      local a = io.line("Front side [Enter = keep; fore, aft, port, starboard, +x, -x, +z, -z]: ")
      if a and a ~= "" then
        local ax2, e = A.resolveSide(af, a)
        if ax2 then
          axis, src = ax2, (ax2 == (A.defaultFront(af.rotors)) and "fore" or "manual")
        else say("not understood (" .. tostring(e) .. "): keeping " .. axis) end
      end
    end
    local fr = A.setFront(af, axis, io.prevFront and io.prevFront.nose, src)
    say(string.format("  front = body %s, flies legs %s; the front-side rotors carry %.0f%% of the weight", fr.axis, fr.nose and "nose-first" or "in any heading", 100 * (fr.share or 0)))
    say("  (change later with `ap front <side> [nose|nonose]`, e.g. when the weight distribution of the ship has changed)")
  end

  if not ask("Is the ship standing on the ground, clear of people and blocks, with the propellers free to turn?") then
    return fail("not confirmed")
  end

  local pcallOk, err = pcall(function()
    -- 2a. motor -> bearing (all transmissions on, one motor at a time)
    local anyTrans = false
    for _, r in ipairs(af.rotors) do if r.hasTrans then anyTrans = true end end
    for j = 1, n do if af.rotors[j].hasTrans then hal.setLevel(j, SPIN_LEVEL) end end
    local newMotor = {}
    for j = 1, n do
      local rpm = zeros(n)
      rpm[j] = SPIN
      hal.actuate(rpm, nil)
      settle(hal)
      local t = turning(hal, n)
      if #t ~= 1 then error(string.format("motor %s turned %d bearings (expected exactly 1)", tostring(af.rotors[j].motor), #t), 0) end
      newMotor[t[1]] = af.rotors[j].motor
      if t[1] ~= j then report.fixes[#report.fixes + 1] = string.format("motor %s drives %s (not %s)", af.rotors[j].motor, af.rotors[t[1]].zone, af.rotors[j].zone) end
    end
    hal.actuate(zeros(n), nil)
    for j = 1, n do af.rotors[j].motor = newMotor[j] end

    -- 2b. transmission -> bearing (motors on, one transmission at a time)
    if anyTrans then
      for j = 1, n do if af.rotors[j].hasTrans then hal.setLevel(j, 0) end end
      local all = {}
      for j = 1, n do all[j] = SPIN end
      hal.actuate(all, nil)
      settle(hal)
      if #turning(hal, n) ~= 0 then error("bearings turn with every transmission at 0", 0) end
      local newTrans = {}
      for j = 1, n do
        if af.rotors[j].hasTrans then
          hal.setLevel(j, SPIN_LEVEL)
          settle(hal)
          local t = turning(hal, n)
          if #t ~= 1 then error(string.format("transmission %s moved %d bearings (expected exactly 1)", tostring(af.rotors[j].trans), #t), 0) end
          newTrans[t[1]] = af.rotors[j].trans
          if t[1] ~= j then report.fixes[#report.fixes + 1] = string.format("transmission %s drives %s (not %s)", af.rotors[j].trans, af.rotors[t[1]].zone, af.rotors[j].zone) end
          hal.setLevel(j, 0)
        end
      end
      hal.actuate(zeros(n), nil)
      for j = 1, n do if af.rotors[j].hasTrans then af.rotors[j].trans = newTrans[j] end end
    end

    -- 3. every bearing must push UP
    for j = 1, n do if af.rotors[j].hasTrans then hal.setLevel(j, SPIN_LEVEL) end end
    local all = {}
    for j = 1, n do all[j] = SPIN end
    hal.actuate(all, nil)
    settle(hal)
    for j = 1, n do
      local p = hal.probe(j)
      if not p.vec then error(af.rotors[j].zone .. ": cannot read the thrust direction", 0) end
      if p.vec.y < 0.3 then
        local other = (p.handed == "left_handed") and "right_handed" or "left_handed"
        hal.setHanded(j, other)
        settle(hal)
        p = hal.probe(j)
        report.fixes[#report.fixes + 1] = string.format("%s pushed down: switched to %s", af.rotors[j].zone, other)
        if p.vec.y < 0.3 then error(af.rotors[j].zone .. " cannot be made to push up", 0) end
      end
      af.rotors[j].handed = p.handed
    end

    -- 4. thrust calibration at two small speeds (total lift stays below ~25 % of the weight)
    local weight = af.mass * af.g
    local function measure(rpms)
      hal.actuate(rpms, nil)
      settle(hal)
      local k, tot = {}, 0
      for j = 1, n do
        local p = hal.probe(j)
        local sp = math.abs(p.speed or 0)
        if sp < 2 then error(af.rotors[j].zone .. ": bearing is not turning during calibration", 0) end
        k[j] = math.abs(p.thrust or 0) / sp
        tot = tot + math.abs(p.thrust or 0)
      end
      return k, tot
    end
    local lvl = SPIN_LEVEL
    local m1 = {}
    for j = 1, n do m1[j] = 10 end
    local k1, tot1 = measure(m1)
    if tot1 > 0.4 * weight then error(string.format("lift is already %.0f%% of the weight at the first calibration step: aborting", 100 * tot1 / weight), 0) end
    local m2 = {}
    for j = 1, n do
      local b2 = 0.25 * weight / n / k1[j]                 -- bearing RPM giving 25 % of the weight over all rotors
      local mot = af.rotors[j].hasTrans and b2 * 15 / lvl or b2
      m2[j] = math.max(10, math.min(150, math.floor(mot)))
    end
    local k2, tot2 = measure(m2)
    if tot2 > 0.4 * weight then error(string.format("lift %.0f%% of the weight at the second step: aborting", 100 * tot2 / weight), 0) end
    hal.actuate(zeros(n), nil)
    for j = 1, n do
      af.rotors[j].kT = k2[j]
      report[j] = { k1 = k1[j], k2 = k2[j] }
    end
    report.liftFrac = tot2 / weight
  end)
  -- always leave everything stopped
  pcall(function()
    for j = 1, n do if af.rotors[j].hasTrans then hal.setLevel(j, 0) end end
    hal.actuate(zeros(n), nil)
  end)
  pcall(hal.stop)
  if not pcallOk then return fail(tostring(err)) end

  -- 5. levels, limits
  local okp, msg = A.plan(af, al.u)
  if not okp then return fail(msg) end
  if msg ~= "" then say("note: " .. msg) end

  for _, f in ipairs(report.fixes) do say("fixed: " .. f) end
  say(string.format("calibration lift reached %.0f%% of the weight", 100 * (report.liftFrac or 0)))
  for j, r in ipairs(af.rotors) do
    say(string.format("  %-16s kT %.2f thrust/RPM  level %s  hover motor RPM %.0f  max thrust %.0f%%  of weight share", r.zone, r.kT, tostring(r.level), r.hoverRpm,
      100 * r.tmax / (af.mass * af.g * al.u[j])))
  end
  local kmin, kmax = math.huge, 0
  for _, r in ipairs(af.rotors) do kmin, kmax = math.min(kmin, r.kT), math.max(kmax, r.kT) end
  if kmax / kmin > 1.25 then say(string.format("note: rotor thrust constants differ by %.0f%% (propellers not identical?)", 100 * (kmax / kmin - 1))) end
  af.wizardVersion = 1
  af.stamp = tostring(os.epoch and os.epoch("utc") or os.time())     -- a new calibration invalidates the saved trim and tune
  af.tune = nil
  af.calibratedMass = af.mass
  if not io.nosave then
    local ok, e = A.save(af, io.path)
    if not ok then return nil, e end
    say("saved " .. (io.path or A.PATH))
  end
  return af, report
end

return W
]=],
  ["ap/main.lua"] = [[
-- ap/main.lua : command dispatcher for the general multirotor autopilot ("ap"). Everything runs inside ComputerCraft.
local args = { ... }
package.path = package.path .. ";/ap/lib/?.lua;/ap/?.lua"

-- a trailing `yes` answers the "area clear?" question (use only when the area is already confirmed clear, e.g. driven remotely)
local assumeYes = false
local rangeArg
for _ = 1, 2 do                                  -- trailing words: `yes` and/or `range=N`, in any order
  local last = args[#args]
  if last == "yes" then assumeYes = true; args[#args] = nil
  elseif type(last) == "string" and last:match("^range=%d+$") then rangeArg = tonumber(last:match("%d+")); args[#args] = nil end
end

-- leading `key=value` words override a numeric setting for this run (controller or navigation settings), e.g. `ap wnAtt=0.8 maxLag=5 goto 10 20`
local overrides = {}
while args[1] and args[1]:match("^[%a_][%w_]*=%-?[%d%.]+$") do
  local k, v = args[1]:match("^([%a_][%w_]*)=(.+)$")
  overrides[k] = tonumber(v)
  table.remove(args, 1)
end
local cmd = args[1]
local H = require("hw")

local function f1(v) return v and string.format("%.1f", v) or "?" end

local function scan()
  local info = H.scan()
  print(string.format("ship mass %s, centre of mass %s", f1(info.mass), info.com and (f1(info.com.x) .. "," .. f1(info.com.y) .. "," .. f1(info.com.z)) or "?"))
  print(string.format("%d motors, %d bearings, %d transmissions", info.nMotors or 0, info.nBearings or 0, info.nTrans or 0))
  for _, r in ipairs(info.rotors) do
    local p = r.pos
    print(string.format("  %-16s pos(x%s y%s z%s) %s motor=%s%s trans=%s", r.zone,
      p and f1(p.x) or "?", p and f1(p.y) or "?", p and f1(p.z) or "?", tostring(r.handed),
      tostring(r.motor), r.link and (" [" .. r.link .. "]") or "", tostring(r.trans)))
  end
  for _, n in ipairs(info.notes) do print("note: " .. n) end
  local bad = H.check(info)
  for _, w in ipairs(bad) do printError("problem: " .. w) end
  if #bad == 0 then print("scan OK") end
  return info, bad
end

local function yes(q)
  if assumeYes then return true end
  write(q .. " [y/N] ")
  local a = read()
  return a and a:lower():sub(1, 1) == "y"
end

-- load the airframe, check the ship, fly `points` (absolute world coordinates) and report
local function flyRoute(points, opts, what, tp)
  local A, HAL, F, Nav = require("airframe"), require("hal"), require("fly"), require("nav")
  local af, e = A.load()
  if not af then error(e, 0) end
  local info, bad = scan()
  if #bad > 0 then error("fix the problems above first", 0) end
  local okr, rmsg = A.refresh(af, info)       -- mass, inertia, centre of mass and the transmission levels follow the ship as it is now
  if not okr then error("the ship cannot fly as it is now: " .. tostring(rmsg) .. " (run `ap wizard` if the propellers changed)", 0) end
  if af.calibratedMass and math.abs(info.mass - af.calibratedMass) > 0.1 * af.calibratedMass then
    print(string.format("note: mass %.1f now, %.1f at setup: levels re-planned; the saved trim is ignored until it is learned again", info.mass, af.calibratedMass))
  end
  if af.front and af.front.share then
    local sh = A.frontShare(af, af.front.axis)
    if sh and math.abs(sh - af.front.share) > 0.1 then
      print(string.format("note: the front-side rotors now carry %.0f%% of the weight (%.0f%% when the front was set): the weight distribution changed; `ap front <side>` moves the front if the ship should fly the other way round", 100 * sh, 100 * af.front.share))
    end
  end
  local p = sublevel.getLogicalPose().position
  -- CHUNK SAFETY (learned 2026-11-16: the tricopter's computer stopped mid-flight when the ship left the loaded area): refuse routes that
  -- go farther from the start than `range` metres (default 100) unless the user says the whole route is loaded: `range=N` as the last word
  local range = rangeArg or 100
  local far = 0
  if tp then far = math.sqrt((tp.X - p.x) ^ 2 + (tp.Z - p.z) ^ 2)
  else for _, w in ipairs(points) do far = math.max(far, math.sqrt(((w.x or w[1]) - p.x) ^ 2 + ((w.z or w[2]) - p.z) ^ 2)) end end
  if far > range then
    error(string.format("the route goes %.0f m from the start, more than the %.0f m range that stays in the loaded chunks. Stay near the ship, or add `range=%d` as the last word if the whole route is loaded.", far, range, math.ceil(far)), 0)
  end
  if tp then
    local ok = tp.X and tp.Z
    if not ok then error("transit needs X and Z", 0) end
    local dist = math.sqrt((tp.X - p.x) ^ 2 + (tp.Z - p.z) ^ 2)
    print(string.format("%s: %.0f m, cruise %s m above the start, up to %.0f m/s, floor %s m, lean %s deg", what, dist, tostring(tp.cruise or "auto (from the air pressure profile)"), tp.vmax or 20, tostring(tp.floor or "auto"), tostring(tp.lean or 0)))
    if not yes("The ship will climb, fly fast to the target, descend and land there. The whole path (and a stopping distance beyond) must be clear. Go?") then error("cancelled", 0) end
  else
    local route, rerr = Nav.build(points, p)
    if not route then error(rerr, 0) end
    print(string.format("%s: %d waypoint(s), about %.0f s", what, #route, Nav.estimate(route, p)))
    if not yes("The ship will take off and fly this route, then land at the end. Area clear?") then error("cancelled", 0) end
  end
  local hal = HAL.real(af)
  local rows = { "t,phase,y,vy,tilt,x,z,refx,refz,vx,vz,refvx,refvz,rpm1,rpm2,rpm3,rpm4,rpm5,rpm6" }
  opts.say = print
  opts.range = rangeArg
  local Ctl, NavM = require("ctl"), require("nav")
  opts.ctlOver, opts.navOver = {}, {}
  for k, v in pairs(overrides) do
    if Ctl.defaults[k] ~= nil then opts.ctlOver[k] = v
    elseif NavM.defaults[k] ~= nil then opts.navOver[k] = v
    else error("unknown setting " .. k .. "=: not a controller or navigation setting", 0) end
  end
  opts.maxTilt = opts.maxTilt or (tp and 35 or 25)
  opts.log = function(t, st, rpm, phase, info2, ex)
    local r = {}
    for j, v in ipairs(rpm) do r[j] = string.format("%.0f", v) end
    rows[#rows + 1] = string.format("%.2f,%s,%.3f,%.3f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%s", t, phase, st.y, st.vy,
      math.acos(math.min(1, st.cosTilt)) * 57.2958, st.px, st.pz, ex.ref.px, ex.ref.pz, st.vx, st.vz, ex.ref.vx, ex.ref.vz, table.concat(r, ","))
  end
  local res, extra
  if tp then res, extra = F.transit(hal, af, tp, opts) else res = F.route(hal, af, points, opts) end
  pcall(hal.stop)
  local okLog, logErr = A.saveLog("/ap_data/flight.csv", rows)
  if not okLog then print("flight log not saved: " .. tostring(logErr)) end
  print(string.format("gyro check: API angular velocity matches the attitude change best as '%s' (score %.2f)", tostring(res.gyro), res.gyroScore or 0))
  for _, w in ipairs(res.waypoints or {}) do print(string.format("  waypoint %d reached at t=%.1f, %.2f m off", w.i, w.t, w.err)) end
  if tp and res.brakeScale and res.ok then
    -- remember what the ship really manages when braking: the next transit plans with it from the start
    af.tune = af.tune or {}
    af.tune.planBrake = math.floor(res.brakeScale * 100 + 0.5) / 100
    A.save(af)
  end
  if tp and res.coast and res.ok and res.coast.kGround and res.coast.kGround > 0 then
    -- the measured drag (per m, at the ground pressure) replaces the guess the planner and the cruise-height search use
    af.tune = af.tune or {}
    af.tune.dragK = tonumber(string.format("%.3g", res.coast.kGround))
    A.save(af)
    print(string.format("stored dragK %s for this ship (`ap set dragK default` to forget it)", tostring(af.tune.dragK)))
  end
  if tp then
    print(string.format("transit: floor %.0f m (lowest clearance %.1f m, %d breach steps), max tilt %.1f deg, time %.1f s", extra and extra.floor or 0, res.minClear or 0, res.floorBreaches or 0, res.maxTilt or 0, res.t or 0))
  end
  print(string.format("flight %s: %s, max tilt %.1f deg, max distance from reference %.2f m, hold rms %s m, landed %s m above the start height%s",
    res.ok and "OK" or "FAULT", tostring(res.reason), res.maxTilt or 0, res.maxDrift or 0,
    res.holdRms and string.format("%.3f", res.holdRms) or "-", res.landedAgl and string.format("%.2f", res.landedAgl) or "-",
    res.final and string.format(", ended at x %.1f z %.1f", res.final.x, res.final.z) or ""))
end

if cmd == "scan" then
  scan()
elseif cmd == "selftest" then
  local ok = require("selftest")(args[2])
  if not ok then error("selftest failed", 0) end
elseif cmd == "wizard" then
  -- ground-only setup: spins each rotor slowly, never lifts the ship
  local info, bad = scan()
  if #bad > 0 then error("fix the problems above first", 0) end
  local W, HAL = require("wizard"), require("hal")
  local af = W.skeleton(info)
  local hal = HAL.real(af)
  local old = require("airframe").load()
  local io2 = { say = print, ask = yes, prevFront = old and old.front }
  if not assumeYes then io2.line = function(q) write(q) return read() end end
  local af2, e = W.run(hal, af, io2)
  if not af2 then error(tostring(e), 0) end
elseif cmd == "fly" then
  -- ap fly [height=5] [secs=10]: take off straight up, hold, land (hover test)
  local height, secs = tonumber(args[2]) or 5, tonumber(args[3]) or 10
  local p = sublevel.getLogicalPose().position
  flyRoute({ { x = p.x, z = p.z, y = p.y + height, dwell = secs } }, {}, string.format("hover at %.1f m for %d s", height, secs))
elseif cmd == "tune" then
  -- ap tune: hover at 3 m and measure torque authority, actuator delay and per-direction lateral authority (stored in the airframe)
  local A, HAL, Tn = require("airframe"), require("hal"), require("tune")
  local af, e = A.load()
  if not af then error(e, 0) end
  local info, bad = scan()
  if #bad > 0 then error("fix the problems above first", 0) end
  local okr, rmsg = A.refresh(af, info)
  if not okr then error(tostring(rmsg), 0) end
  if not yes("The ship will take off to 3 m and wobble and push sideways for about a minute, then land. Area clear (10 m around)?") then error("cancelled", 0) end
  local hal = HAL.real(af)
  local rows = { "t,phase,x,z,y,vx,vz,tilt,refx,refz,rpm1,rpm2,rpm3,rpm4" }
  local tuneLog = function(t, st, rpm, phase, info, ex)
    local r = {}
    for j = 1, 4 do r[j] = rpm[j] and string.format("%.0f", rpm[j]) or "" end
    rows[#rows + 1] = string.format("%.2f,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.3f,%.3f,%s", t, phase, st.px, st.pz, st.y, st.vx, st.vz,
      math.acos(math.min(1, st.cosTilt)) * 57.2958, ex.ref.px, ex.ref.pz, table.concat(r, ","))
  end
  local tune, why, res = Tn.run(hal, af, { say = print, log = tuneLog })
  pcall(hal.stop)
  A.saveLog("/ap_data/tune.csv", rows)
  if not tune then error("tune failed: " .. tostring(why) .. " (log: /ap_data/tune.csv)", 0) end
  af.tune = tune
  local ok, werr = A.save(af)
  print(string.format("authority roll %.2f pitch %.2f yaw %.2f   delay %.2f s", tune.gx, tune.gz, tune.gy, tune.delay))
  print(string.format("lateral authority +x %.2f -x %.2f +z %.2f -z %.2f   (body axes: x along the plot grid x)", tune.lat.xp, tune.lat.xn, tune.lat.zp, tune.lat.zn))
  print(ok and "saved to the airframe" or ("could not save: " .. tostring(werr)))
elseif cmd == "trim" then
  -- ap trim [secs=20]: hover at 2 m so the integrators learn the ship's hidden imbalances; saved for later flights (same mass)
  local secs = tonumber(args[2]) or 20
  local p = sublevel.getLogicalPose().position
  flyRoute({ { x = p.x, z = p.z, y = p.y + 2, dwell = secs } }, {}, string.format("trim hover for %d s", secs))
elseif cmd == "cleartrim" then
  if fs.exists("/ap_data/trim.json") then fs.delete("/ap_data/trim.json") print("trim cleared") else print("no saved trim") end
elseif cmd == "showtune" then
  local A = require("airframe")
  local af, e = A.load()
  if not af then error(e, 0) end
  if af.tune then print(textutils.serialise(af.tune)) else print("no tune data: run `ap tune`") end
  local tr = fs.exists("/ap_data/trim.json") and fs.open("/ap_data/trim.json", "r")
  if tr then print("trim: " .. tr.readAll()) tr.close() else print("no saved trim") end
elseif cmd == "transit" then
  -- ap transit X Z [cruise] [vmax=15] [accel=2] [floor] [lean=0]: fast point-to-point. cruise/floor are metres above the start (defaults from the ship size),
  -- lean is the maximum whole-ship lean in degrees (0 = bearings only).
  local X, Z = tonumber(args[2]), tonumber(args[3])
  if not (X and Z) then error("usage: ap transit X Z [cruise] [vmax=15] [accel=2] [floor] [lean=0]", 0) end
  local tp = { X = X, Z = Z, cruise = tonumber(args[4]), vmax = tonumber(args[5]) or 15, accel = tonumber(args[6]) or 2.0, floor = tonumber(args[7]), lean = tonumber(args[8]) or 0 }
  flyRoute({}, {}, string.format("transit to %.0f, %.0f", X, Z), tp)
elseif cmd == "where" then
  local p = sublevel.getLogicalPose()
  local v = sublevel.getLinearVelocity()
  local M = require("m")
  local o = p.orientation
  local at = M.attitude({ w = o.a, x = o.v.x, y = o.v.y, z = o.v.z })
  print(string.format("position x %.2f  y %.2f  z %.2f   velocity %.2f %.2f %.2f (ground speed %.2f m/s)   heading %.1f deg  tilt %.1f deg",
    p.position.x, p.position.y, p.position.z, v.x, v.y, v.z, math.sqrt(v.x * v.x + v.z * v.z), at.psi, at.tilt))
  local Af = require("airframe")
  local af0 = Af.load()
  if af0 then print(string.format("front (body %s) points to compass %.0f deg", Af.front(af0).axis, Af.frontCompass(af0, { w = o.a, x = o.v.x, y = o.v.y, z = o.v.z }))) end
elseif cmd == "set" then
  -- ap set <name> [value]: store / show a per-ship planning value in the airframe (leanInflow: m/s of airflow through the rotors the lean may
  -- cause, default 2; dragK: drag per m of the ship, default 0.0002). `ap set name default` removes it.
  local A = require("airframe")
  local af, e = A.load()
  if not af then error(e, 0) end
  local name, val = args[2], args[3]
  local allowed = { leanInflow = true, dragK = true, planBrake = true }
  af.tune = af.tune or {}
  if name then
    if not allowed[name] then error("settable: leanInflow, dragK, planBrake", 0) end
    if val == "default" then af.tune[name] = nil
    elseif tonumber(val) then af.tune[name] = tonumber(val)
    elseif val then error("value must be a number or `default`", 0) end
    if val then local ok, werr = A.save(af) if not ok then error(tostring(werr), 0) end print("saved") end
  end
  for k in pairs(allowed) do print(k .. " = " .. tostring(af.tune[k])) end
elseif cmd == "front" then
  -- ap front                       show the front of the ship
  -- ap front <side> [nose|nonose]  fore | aft | port | starboard (relative to the avionics controller) or +x -x +z -z (body axes); default: keep nose setting
  local A = require("airframe")
  local af, e = A.load()
  if not af then error(e, 0) end
  local side, noseArg = args[2], nil
  for i = 2, #args do
    if args[i] == "nose" then noseArg = true elseif args[i] == "nonose" then noseArg = false end
  end
  if side == "nose" or side == "nonose" then side = nil end
  if side or noseArg ~= nil then
    local axis = A.front(af).axis
    if side then
      local ax2, err = A.resolveSide(af, side)
      if not ax2 then error(err, 0) end
      axis = ax2
    end
    local fr = A.setFront(af, axis, noseArg, (side and side:lower() == "fore") and "fore" or "manual")
    local ok, werr = A.save(af)
    if not ok then error(tostring(werr), 0) end
    print("saved. The saved trim and tune stay valid (the ship itself did not change).")
  end
  local fr = A.front(af)
  local foreAxis = A.defaultFront(af.rotors)
  local share = A.frontShare(af, fr.axis)
  print(string.format("front = body %s (%s)%s, legs are flown %s", fr.axis, fr.src == "fore" and "the fore side of the avionics controller" or "chosen by hand",
    fr.axis == foreAxis and "" or ("; the controller's fore is body " .. foreAxis), fr.nose and "nose-first" or "in any heading"))
  if share then print(string.format("the front-side rotors carry %.0f%% of the weight now%s", 100 * share, fr.share and string.format(" (%.0f%% when the front was set)", 100 * fr.share) or "")) end
  local pose = sublevel and sublevel.getLogicalPose and sublevel.getLogicalPose()
  if pose then
    local o = pose.orientation
    print(string.format("the front points to compass %.0f deg now (0 north, 90 east)", A.frontCompass(af, { w = o.a, x = o.v.x, y = o.v.y, z = o.v.z })))
  end
elseif cmd == "land" then
  local height, secs = tonumber(args[2]) or 3, tonumber(args[3]) or 6
  local p = sublevel.getLogicalPose().position
  flyRoute({ { x = p.x, z = p.z, y = p.y + height, dwell = secs } }, {}, string.format("hover at %.1f m for %d s, then land", height, secs))
elseif cmd == "climb" then
  -- ap climb peak [secs]: climb to peak metres above the start, dwell, descend to 1 m, land
  local peak, secs = tonumber(args[2]) or 4, tonumber(args[3]) or 3
  local p = sublevel.getLogicalPose().position
  flyRoute({ { x = p.x, z = p.z, y = p.y + peak, dwell = secs }, { x = p.x, z = p.z, y = p.y + 1, dwell = 1 } }, {}, string.format("climb to %.1f m and come down", peak))
elseif cmd == "dry" then
  -- ap dry [secs]: read the sensors and compute what would be commanded; NEVER actuates
  local A, HAL, F = require("airframe"), require("hal"), require("fly")
  local af, e = A.load()
  if not af then error(e, 0) end
  local info, bad = scan()
  if #bad > 0 then error("fix the problems above first", 0) end
  local okr, rmsg = A.refresh(af, info)
  if not okr then error(tostring(rmsg), 0) end
  local secs = tonumber(args[2]) or 10
  local real = HAL.real(af)
  local dry = {}
  for k, v in pairs(real) do dry[k] = v end
  dry.actuate = function() end
  dry.cycle = function() sleep(0.05) return real.sense() end
  dry.stop = function() end
  dry.setLevel = function() end
  dry.setHanded = function() end
  local last = -1
  local p = sublevel.getLogicalPose().position
  F.route(dry, af, { { x = p.x, z = p.z, y = p.y + 2, dwell = 1 } }, {
    maxSeconds = secs, ramp = 1, lockPath = "/ap_data/dry.lock", trim = false,
    log = function(t, st, rpm, phase)
      if t - last >= 1 then
        last = t
        local r = {}
        for j, v in ipairs(rpm) do r[j] = string.format("%.0f", v) end
        print(string.format("t=%.0f %s  y %.2f  vy %.2f  tilt %.1f  would command RPM %s", t, phase, st.y, st.vy, math.acos(math.min(1, st.cosTilt)) * 57.2958, table.concat(r, " ")))
      end
    end,
  })
  print("dry run finished: nothing was actuated")
elseif cmd == "calib" then
  local info, bad = scan()
  if #bad > 0 then error("fix the problems above first", 0) end
  local W, HAL = require("wizard"), require("hal")
  local af = W.skeleton(info)
  local af2, e = W.run(HAL.real(af), af, { say = print, ask = yes })
  if not af2 then error(tostring(e), 0) end
elseif cmd == "pushmap" or cmd == "gust" or cmd == "imbal" or cmd == "leanstep" or cmd == "yawcal" or cmd == "idpulse" then
  -- diagnostics (diag.lua / tune.lua): they hover at about 4 m and run a timed script; CSV in /ap_data/diag_<name>.csv
  local A, HAL, D, Tn = require("airframe"), require("hal"), require("diag"), require("tune")
  local af, e = A.load()
  if not af then error(e, 0) end
  local info, bad = scan()
  if #bad > 0 then error("fix the problems above first", 0) end
  local okr, rmsg = A.refresh(af, info)
  if not okr then error(tostring(rmsg), 0) end
  if not yes("The ship will take off to about 4 m and run the " .. cmd .. " test (up to two minutes), then land. Area clear (15 m around)?") then error("cancelled", 0) end
  local hal = HAL.real(af)
  local opts = { say = print, height = 4 }
  local out
  if cmd == "idpulse" then
    local tune, why = Tn.run(hal, af, opts)
    pcall(hal.stop)
    if not tune then error("idpulse failed: " .. tostring(why), 0) end
    print(string.format("authority roll %.2f pitch %.2f yaw %.2f, delay %.2f s (not saved; use `ap tune` to store)", tune.gx, tune.gz, tune.gy, tune.delay))
  else
    out = D[cmd](hal, af, opts)
    pcall(hal.stop)
    local res = out.flight
    print(string.format("flight %s: %s, max tilt %.1f deg", res.ok and "OK" or "FAULT", tostring(res.reason), res.maxTilt or 0))
    if cmd == "pushmap" then
      print("body direction (deg from +x toward +z)   achieved/commanded   sideways")
      for _, r in ipairs(out.table) do print(string.format("  %3d   %5.2f   %+5.2f", r.deg, r.gain or 0, r.perp or 0)) end
      print(#out.weak > 0 and ("weak directions (<0.8): " .. table.concat(out.weak, ", ") .. " deg: `ap tune` stores per-axis gains") or "no weak direction")
    elseif cmd == "gust" then
      for _, t in ipairs(out.tests) do
        print(string.format("  %-36s max dev %s  max tilt %.1f  settles in %.1f s", t.name, t.maxDev and string.format("%.2f m", t.maxDev) or (t.maxYaw and string.format("%.1f deg", t.maxYaw)) or string.format("%.2f m alt", t.maxDy or 0), t.maxTilt or 0, t.settle or 0))
      end
    elseif cmd == "imbal" then
      print(string.format("injected %+d%% on rotor 1, %+d%% on the last rotor, %+d%% on all; peak tilt %.1f deg, peak position error %.2f m, peak altitude error %.2f m",
        out.injected.rotor1, out.injected.rotorN, out.injected.all, out.peak.tilt, out.peak.dev, out.peak.dy))
      for _, ts in ipairs(out.times) do
        local s = out.learned[ts]
        if s then print(string.format("  at t=%.0f s learned: roll %.3f  pitch %.3f  yaw %.3f  altitude %.3f", ts, s.ix or 0, s.iz or 0, s.iy or 0, s.ia or 0)) end
      end
    elseif cmd == "leanstep" then
      for _, s in ipairs(out.steps) do print(string.format("  commanded %+.1f m/s2: achieved %s m/s2, per unit of actual lean %s, lean followed %s of the command, peak tilt %.1f deg", s.cmd, s.achieved and string.format("%+.2f", s.achieved) or "?", s.effect and string.format("%.2f", s.effect) or "?", s.follow and string.format("%.0f%%", 100 * s.follow) or "?", s.tilt)) end
    elseif cmd == "yawcal" then
      for _, r in ipairs(out.rotors) do print(string.format("  rotor %-16s yaw change %+.2f deg/s, %s of the prediction", tostring(r.rotor), r.dYaw, r.ratio and string.format("%.2f", r.ratio) or "?")) end
    end
    print("log: /ap_data/diag_" .. cmd .. ".csv")
  end
elseif cmd == "clean" then
  -- ap clean: delete the CSV logs and a leftover installer to free disk space (the computer has about 1 MB)
  local n = 0
  if fs.exists("/ap_data") then
    for _, f in ipairs(fs.list("/ap_data")) do
      if f:match("%.csv$") then fs.delete("/ap_data/" .. f) n = n + 1 end
    end
  end
  if fs.exists("/apinstall.lua") then fs.delete("/apinstall.lua") n = n + 1 end
  print(string.format("removed %d file(s); %d bytes free", n, fs.getFreeSpace("/")))
elseif cmd == "recover" then
  -- ap recover: run at boot (startup.lua). If a flight was interrupted (lock file present) it lands the ship; otherwise it does nothing.
  local A, HAL, F = require("airframe"), require("hal"), require("fly")
  if not fs.exists(F.LOCK) then
    print("ap: no interrupted flight")
  else
    print("ap: INTERRUPTED FLIGHT found - taking control and landing")
    local af, e = A.load()
    if not af then error(e, 0) end
    local info = H.scan()
    af.mass, af.inertia = info.mass or af.mass, info.inertia or af.inertia
    local hal = HAL.real(af)
    local res = F.recover(hal, af, { say = print, maxTilt = 35 })
    pcall(hal.stop)
    print(string.format("recovery %s: %s", res.ok and "OK" or "FAULT", tostring(res.reason)))
  end
elseif cmd == "goto" then
  -- ap goto X Z [Y] [speed]: fly to world coordinates (Y absolute, default 3 m above here), land there
  local X, Z, Y, V = tonumber(args[2]), tonumber(args[3]), tonumber(args[4]), tonumber(args[5])
  if not (X and Z) then error("usage: ap goto X Z [Y] [speed]   (world coordinates; Y default: 3 m above here)", 0) end
  flyRoute({ { x = X, z = Z, y = Y, speed = V } }, {}, string.format("goto %.1f, %.1f", X, Z))
elseif cmd == "move" then
  -- ap move dx dz [height=3] [speed]: fly dx/dz metres from here (world axes), dwell, fly back, land
  local dx, dz, h, V = tonumber(args[2]) or 0, tonumber(args[3]) or 0, tonumber(args[4]) or 3, tonumber(args[5])
  local p = sublevel.getLogicalPose().position
  flyRoute({ { x = p.x + dx, z = p.z + dz, y = p.y + h, speed = V } }, { home = true }, string.format("move %.1f,%.1f and back", dx, dz))
elseif cmd == "wp" then
  -- ap wp [file] [home]: waypoints.json = {"speed":5,"dwell":2,"home":true,"points":[{"x":..,"z":..,"y":..,"speed":..,"dwell":..}]}
  local file = args[2] or "waypoints.json"
  if not fs.exists(file) then error("no waypoint file " .. file, 0) end
  local f = fs.open(file, "r")
  local ok, data = pcall(textutils.unserialiseJSON, f.readAll())
  f.close()
  if not ok or type(data) ~= "table" or type(data.points) ~= "table" or #data.points == 0 then error("bad waypoint file (need {\"points\":[{x,z,y}...]})", 0) end
  local pts = {}
  for i, w in ipairs(data.points) do pts[i] = { x = w.x, z = w.z, y = w.y, speed = w.speed or data.speed, dwell = w.dwell or data.dwell } end
  flyRoute(pts, { home = (args[3] == "home") or (data.home == true) }, "waypoints from " .. file)
elseif cmd == "cruise" then
  -- ap cruise speed heading secs [height=3]: hold a speed on a compass heading (0 north, 90 east) for secs, stop, land
  local V, Hd, T, h = tonumber(args[2]), tonumber(args[3]), tonumber(args[4]), tonumber(args[5]) or 3
  if not (V and Hd and T) then error("usage: ap cruise speed heading secs [height]   (heading: 0 north, 90 east, 180 south, 270 west)", 0) end
  local ux, uz = require("nav").compassVec(Hd)
  local p = sublevel.getLogicalPose().position
  flyRoute({ { x = p.x + ux * V * T, z = p.z + uz * V * T, y = p.y + h, speed = V } }, {}, string.format("cruise %.1f m/s heading %d for %d s", V, Hd, T))
else
  print("ap: general multirotor autopilot (work in progress)")
  print("  ap scan                     find rotors and their positions")
  print("  ap selftest                 simulator checks (nothing flies)")
  print("  ap wizard                   ground setup: verify pairing, thrust direction, calibrate (slow spins only)")
  print("  ap fly [h] [s]              hover test: take off to h m, hold s seconds, land (ap land is the same)")
  print("  ap dry [secs]               read the sensors, print what WOULD be commanded; never actuates")
  print("  ap where                    position, velocity, ground speed, heading")
  print("  ap climb peak [secs]        climb to peak m, dwell, come down to 1 m, land")
  print("  ap pushmap|gust|imbal|leanstep|yawcal|idpulse   diagnostic test flights (CSV in /ap_data)")
  print("  ap key=value ... <command>  override a numeric controller/navigation setting for one run")
  print("  ap tune                     measure authority, delay and lateral authority per direction (in flight)")
  print("  ap trim [secs]              hover and learn the ship's imbalances (saved)")
  print("  ap showtune / cleartrim     show or clear the stored tune and trim")
  print("  ap goto X Z [Y] [speed]     fly to world coordinates and land there")
  print("  ap recover                  land the ship after an interrupted flight (run it from startup.lua)")
  print("  ap clean                    delete CSV logs and a leftover installer to free disk space")
  print("  ap transit X Z [cruise] [vmax] [accel] [floor] [lean]   fast flight above a safety floor")
  print("  ap move dx dz [h] [speed]   fly dx/dz metres away and back, land")
  print("  ap wp [file] [home]         fly the waypoints in a JSON file")
  print("  ap cruise v heading secs    hold a speed on a compass heading")
  print("  ap set [leanInflow|dragK|planBrake] [value]  planning values of this ship (see README)")
  print("  ap front [side] [nose|nonose]  show / set the front of the ship (fore, aft, port, starboard, +x, -x, +z, -z)")
  print("  (add `yes` as the last word to skip the 'area clear?' question)")
end
]],
  ["ap/selftest.lua"] = [=[
-- ap/selftest.lua : `ap selftest` - pure-logic checks (no peripherals, nothing flies). Runs inside ComputerCraft.
local M = require("m")
local H = require("hw")
local A = require("alloc")
local Sim = require("sim")
local Ctl = require("ctl")
local W = require("wizard")
local HAL = require("hal")
local AF = require("airframe")
local F = require("fly")
local Nav = require("nav")
local Tune = require("tune")
local Diag = require("diag")
local AFm = require("airframe")

local T = { pass = 0, fail = 0, quiet = false }
local function check(name, ok, detail)
  if ok then T.pass = T.pass + 1 else
    T.fail = T.fail + 1
    print("FAIL " .. name .. (detail and (": " .. detail) or ""))
  end
end
local function near(a, b, tol) return math.abs(a - b) <= tol end

local function testMath()
  local x = M.solve({ { 2, 1 }, { 1, 3 } }, { 3, 5 })
  check("solve 2x2", x and near(x[1], 0.8, 1e-9) and near(x[2], 1.4, 1e-9))
  check("solve singular", M.solve({ { 1, 2 }, { 2, 4 } }, { 1, 2 }) == nil)
  local s = M.minNorm({ { 1, 1, 0 }, { 0, 1, 1 } }, { 2, 2 })
  check("minNorm satisfies", s and near(s[1] + s[2], 2, 1e-9) and near(s[2] + s[3], 2, 1e-9))
  local q = M.qintegrate({ w = 1, x = 0, y = 0, z = 0 }, { x = 0, y = 1, z = 0 }, math.pi / 2)  -- 90 deg about Y
  local v = M.qrot(q, { x = 1, y = 0, z = 0 })
  check("quaternion yaw 90", near(v.x, 0, 1e-9) and near(v.z, -1, 1e-9), v.x .. "," .. v.z)
  check("attitude psi after 90 deg about +Y", near(M.attitude(q).psi, 90, 1e-6), M.attitude(q).psi)
end

local function testIds()
  local x, y, z = H.decodeId("4e20ff9388408083")
  check("decode quad fore", x == 20481022 and y == 131 and z == 20481032, tostring(x) .. "," .. tostring(y) .. "," .. tostring(z))
  x, y, z = H.decodeId("4e21031388c0a081")
  check("decode tri fore_port", x == 20481036 and y == 129 and z == 20483082, tostring(x) .. "," .. tostring(y) .. "," .. tostring(z))
  check("decode bad id", H.decodeId("zz") == nil and H.decodeId(nil) == nil)
end

local LAYOUTS = { "tri", "quad+", "quadx", "hex", "octo", "offset" }

local function testAlloc()
  for _, name in ipairs(LAYOUTS) do
    local cfg = Sim.config(name)
    local al, err = A.new(cfg.rotors)
    check(name .. " allocator builds", al ~= nil, err)
    if al then
      local W = cfg.mass * cfg.g
      -- exactness: B*T = (Fy, tau_x, tau_z)
      local Tt = A.thrust(al, W, 120, -90)
      local fy, tx, tz = 0, 0, 0
      for j, r in ipairs(cfg.rotors) do fy = fy + Tt[j]; tx = tx - r.z * Tt[j]; tz = tz + r.x * Tt[j] end
      check(name .. " thrust exact", near(fy, W, 1e-6) and near(tx, 120, 1e-6) and near(tz, -90, 1e-6), fy .. "," .. tx .. "," .. tz)
      -- pure collective makes no torque
      local Tc = A.thrust(al, W, 0, 0)
      local ctx, ctz = 0, 0
      for j, r in ipairs(cfg.rotors) do ctx = ctx - r.z * Tc[j]; ctz = ctz + r.x * Tc[j] end
      check(name .. " collective torque-free", near(ctx, 0, 1e-6) and near(ctz, 0, 1e-6))
      -- saturation: demand a big torque, attitude is kept, collective gives way, limits respected
      local Ts, info = A.thrust(al, W, 1e9, 0)
      local okLim = true
      for j, r in ipairs(cfg.rotors) do if Ts[j] < -1e-6 or Ts[j] > r.tmax + 1e-6 then okLim = false end end
      check(name .. " saturated thrusts in limits", okLim and info.saturated)
      local Ts2, info2 = A.thrust(al, W * 1.5, 100, 100)    -- attitude fits, collective must give
      local okLim2, fy2 = true, 0
      for j, r in ipairs(cfg.rotors) do fy2 = fy2 + Ts2[j]; if Ts2[j] < -1e-6 or Ts2[j] > r.tmax + 1e-6 then okLim2 = false end end
      check(name .. " collective gives way before attitude", okLim2 and info2.attScale == 1 and fy2 <= W * 1.5 + 1e-6)
      -- tilts: lateral force + yaw reproduced
      local Th = A.thrust(al, W, 0, 0)
      local tl, ti = A.tilts(al, Th, 50, -30, 40, 0.3)
      check(name .. " tilts exact", tl and near(ti.Fx, 50, 1e-6) and near(ti.Fz, -30, 1e-6) and near(ti.tauY, 40, 1e-6) and ti.scale == 1,
        tl and (ti.Fx .. "," .. ti.Fz .. "," .. ti.tauY) or "nil")
      local tl2, ti2 = A.tilts(al, Th, 1e6, 0, 0, 0.2)
      local worst = 0
      for _, t in ipairs(tl2) do worst = math.max(worst, math.sqrt(t[1] ^ 2 + t[2] ^ 2)) end
      check(name .. " tilt cone cap", worst <= 0.2 + 1e-9 and ti2.scale < 1)
    end
  end
  check("two rotors rejected", A.new({ { x = 1, y = 0, z = 0 }, { x = -1, y = 0, z = 0 } }) == nil)
  check("collinear rejected", A.new({ { x = 1, y = 0, z = 0 }, { x = 2, y = 0, z = 0 }, { x = -3, y = 0, z = 0 } }) == nil)
end

-- closed loop in the simulator. Returns metrics.
local function fly(name, opt)
  opt = opt or {}
  local cfg = Sim.config(name, opt.sim)
  local sim = Sim.new(cfg)
  -- the controller's belief (may differ from the truth to test robustness)
  local belief = { mass = cfg.mass, inertia = { x = cfg.inertia.x * (opt.iErr or 1), y = cfg.inertia.y * (opt.iErr or 1), z = cfg.inertia.z * (opt.iErr or 1) }, rotors = cfg.rotors }
  if opt.gainErr then for j, r in ipairs(cfg.rotors) do r.gainErr = opt.gainErr[j] or 1 end end
  local ctl = Ctl.new(belief)
  local dt = 0.05
  local ref = { y = sim.p.y + (opt.climb or 0), psi = 0 }
  if opt.tiltStart then sim.q = M.qintegrate(sim.q, { x = opt.tiltStart, y = 0, z = 0 }, 1) end
  local maxTilt, ys, samples = 0, 0, 0
  local ymin, ymax = 1e9, -1e9
  local steps = math.floor((opt.secs or 40) / dt)
  for i = 1, steps do
    if i % 25 == 0 and sleep then sleep(0) end      -- CC kills programs that never yield
    local t = i * dt
    if opt.posStep and t > 1 then ref.px, ref.pz = opt.posStep[1], opt.posStep[2] end
    if opt.yawStep and t > 1 then ref.psi = opt.yawStep end
    local s = Ctl.fromSim(sim)
    local Tc, tilt = Ctl.step(ctl, s, ref, dt)
    Sim.step(sim, Tc, tilt, dt)
    local at = M.attitude(sim.q)
    maxTilt = math.max(maxTilt, at.tilt)
    if t > (opt.secs or 40) - 8 then
      samples = samples + 1
      ymin, ymax = math.min(ymin, sim.p.y), math.max(ymax, sim.p.y)
    end
  end
  local at = M.attitude(sim.q)
  return { y = sim.p.y, x = sim.p.x, z = sim.p.z, tilt = at.tilt, psi = at.psi, maxTilt = maxTilt, yref = ref.y, yband = ymax - ymin, v = math.sqrt(sim.v.x ^ 2 + sim.v.y ^ 2 + sim.v.z ^ 2) }
end

local function testFlight()
  for _, name in ipairs(LAYOUTS) do
    local r = fly(name, { secs = 40, climb = 3 })
    check(name .. " climbs 3 m and holds", near(r.y, r.yref, 0.05) and r.yband < 0.05, string.format("y=%.3f ref=%.3f band=%.3f", r.y, r.yref, r.yband))
    check(name .. " stays level", r.tilt < 0.3, string.format("tilt %.2f", r.tilt))
    local r2 = fly(name, { secs = 40, tiltStart = 0.25 })
    check(name .. " recovers from 14 deg tilt", r2.tilt < 0.3 and near(r2.y, r2.yref, 0.3) and r2.maxTilt < 25, string.format("tilt %.2f y %.2f max %.1f", r2.tilt, r2.y - r2.yref, r2.maxTilt))
    local r3 = fly(name, { secs = 40, yawStep = 60 })
    check(name .. " yaws to 60", near(r3.psi, 60, 1.0), string.format("psi %.2f", r3.psi))
    local r4 = fly(name, { secs = 60, posStep = { 12, -8 } })
    check(name .. " position step 12,-8", near(r4.x, 12, 0.3) and near(r4.z, -8, 0.3) and r4.maxTilt < 10, string.format("x %.2f z %.2f max tilt %.1f", r4.x, r4.z, r4.maxTilt))
    -- robustness: wrong inertia belief and one rotor 15 % weak, another 10 % strong
    local ge = {}
    ge[1], ge[2] = 0.85, 1.10
    local r5 = fly(name, { secs = 60, iErr = 1.4, gainErr = ge, climb = 2 })
    check(name .. " robust to inertia x1.4 and rotor gain errors", near(r5.y, r5.yref, 0.1) and r5.tilt < 0.5 and r5.maxTilt < 15, string.format("y %.2f tilt %.2f max %.1f", r5.y - r5.yref, r5.tilt, r5.maxTilt))
  end
end

-- a simulated ship + the (possibly wrong) airframe the software starts with
local function simShip(layout, o)
  o = o or {}
  local cfg = Sim.config(layout, o.sim)
  local n = #cfg.rotors
  for _, r in ipairs(cfg.rotors) do r.tmax = 1e9 end     -- the HAL limits thrust through the motor RPM range
  local info = { mass = cfg.mass, g = cfg.g, inertia = cfg.inertia, rotors = {} }
  local truth = { kT = {}, trans = {}, motorDrives = {}, transDrives = {}, goodHanded = {}, startHanded = {}, liftSign = -1 }
  for j, r in ipairs(cfg.rotors) do
    local b, m, t = "b" .. j, "m" .. j, "t" .. j
    info.rotors[j] = { zone = "rotor" .. j, bearing = b, motor = m, trans = o.trans and t or nil, pos = { x = r.x, y = r.y, z = r.z }, handed = "left_handed" }
    truth.kT[j] = (o.kT or 1.6) * (1 + 0.04 * ((j % 3) - 1))
    truth.trans[j] = o.trans and true or false
    truth.motorDrives[m] = b
    truth.transDrives[t] = b
    truth.goodHanded[j] = "left_handed"
  end
  for _, sw in ipairs(o.swapMotors or {}) do
    truth.motorDrives["m" .. sw[1]], truth.motorDrives["m" .. sw[2]] = truth.motorDrives["m" .. sw[2]], truth.motorDrives["m" .. sw[1]]
  end
  for _, sw in ipairs(o.swapTrans or {}) do
    truth.transDrives["t" .. sw[1]], truth.transDrives["t" .. sw[2]] = truth.transDrives["t" .. sw[2]], truth.transDrives["t" .. sw[1]]
  end
  for _, j in ipairs(o.wrongHand or {}) do truth.startHanded[j] = "right_handed" end
  local af = W.skeleton(info)
  local hal = HAL.sim(cfg, truth, af)
  return hal, af, truth, cfg
end

local QUIET = { say = function() end, ask = function() return true end, path = "/ap_data/selftest_airframe.json" }

local function testWizard()
  -- tricopter (no transmissions): two motors cross-wired, one bearing pushing down
  local hal, af, truth = simShip("tri", { swapMotors = { { 1, 2 } }, wrongHand = { 3 } })
  local res, rep = W.run(hal, af, QUIET)
  check("wizard tri completes", res ~= nil, tostring(rep))
  if res then
    check("wizard tri fixed motor pairing", res.rotors[1].motor == "m2" and res.rotors[2].motor == "m1" and res.rotors[3].motor == "m3")
    check("wizard tri fixed handedness", hal.handed[3] == "left_handed" and res.rotors[3].handed == "left_handed")
    local okk = true
    for j = 1, 3 do if math.abs(res.rotors[j].kT / truth.kT[j] - 1) > 0.02 then okk = false end end
    check("wizard tri thrust constants within 2%", okk, res.rotors[1].kT .. " vs " .. truth.kT[1])
    check("wizard tri direct drive", res.rotors[1].level == nil and res.rotors[1].hoverRpm > 40 and res.rotors[1].hoverRpm < 150, tostring(res.rotors[1].hoverRpm))
    check("wizard tri never lifted off", hal.sim.p.y < 20.001, tostring(hal.sim.p.y))
    local back = AF.load(QUIET.path)
    check("airframe json round trip", back and #back.rotors == 3 and math.abs(back.rotors[2].kT - res.rotors[2].kT) < 1e-9 and back.rotors[1].motor == "m2")
    pcall(fs.delete, QUIET.path)
  end
  -- quad with transmissions: two transmissions cross-wired
  local hal2, af2, truth2 = simShip("quad+", { trans = true, kT = 8.3, swapTrans = { { 2, 3 } } })
  local res2, rep2 = W.run(hal2, af2, QUIET)
  check("wizard quad completes", res2 ~= nil, tostring(rep2))
  if res2 then
    check("wizard quad fixed transmission pairing", res2.rotors[2].trans == "t3" and res2.rotors[3].trans == "t2" and res2.rotors[1].trans == "t1")
    check("wizard quad picks levels 3-5 like the real quad", res2.rotors[1].level >= 3 and res2.rotors[1].level <= 5 and res2.rotors[2].level >= 3 and res2.rotors[2].level <= 5, tostring(res2.rotors[1].level))
    local okk = true
    for j = 1, 4 do if math.abs(res2.rotors[j].kT / truth2.kT[j] - 1) > 0.02 then okk = false end end
    check("wizard quad thrust constants within 2%", okk)
    pcall(fs.delete, QUIET.path)
  end
  -- refusals
  local hal3, af3 = simShip("tri")
  local r3 = W.run(hal3, af3, { say = function() end, ask = function() return false end, nosave = true })
  check("wizard refuses without confirmation", r3 == nil)
  local hal4, af4 = simShip("tri", { kT = 0.3 })       -- far too weak propellers: cannot hover
  local r4 = W.run(hal4, af4, { say = function() end, ask = function() return true end, nosave = true })
  check("wizard rejects props too weak to hover", r4 == nil)
end

local function testFly()
  for _, case in ipairs({ { "tri", {} }, { "quad+", { trans = true, kT = 8.3 } }, { "hex", { kT = 3.0 } } }) do
    local name, o = case[1], case[2]
    local hal, af, truth, cfg = simShip(name, o)
    local res = W.run(hal, af, { say = function() end, ask = function() return true end, nosave = true })
    check(name .. " wizard before flight", res ~= nil)
    if res then
      -- a fresh simulated ship, flown with the airframe the wizard produced
      local halF = HAL.sim(cfg, truth, res)
      local fr = F.hover(halF, res, { height = 5, secs = 10, maxSeconds = 120 })
      check(name .. " hover flight ok", fr.ok, tostring(fr.reason))
      check(name .. " held altitude", fr.holdRms and fr.holdRms < 0.05, tostring(fr.holdRms))
      check(name .. " flight stayed level and put", fr.maxTilt < 6 and fr.maxDrift < 0.6, string.format("tilt %.1f drift %.2f", fr.maxTilt, fr.maxDrift))
      check(name .. " landed gently", fr.landedAgl and fr.landedAgl < 0.2, tostring(fr.landedAgl))
    end
  end
  -- a wrong-signed gyro is caught
  local hal, af, truth, cfg = simShip("tri")
  local res = W.run(hal, af, { say = function() end, ask = function() return true end, nosave = true })
  local halF = HAL.sim(cfg, truth, res)
  local realSense = halF.sense
  halF.sense = function() local s = realSense() s.w = { x = -s.w.x, y = -s.w.y, z = -s.w.z } return s end
  local kickedF = false
  halF.cycle = function(a, b)
    if not kickedF and halF.sim.t > 6 then kickedF = true; halF.sim.w.x = halF.sim.w.x + 0.2; halF.sim.w.z = halF.sim.w.z - 0.15 end
    halF.actuate(a, b)
    return halF.sense()
  end
  local fr = F.hover(halF, res, { height = 5, secs = 5, maxSeconds = 60, maxTilt = 60 })
  check("flight does not depend on the API gyro sign", fr.ok and fr.maxTilt < 12, tostring(fr.reason) .. " tilt " .. tostring(fr.maxTilt))
  check("flipped gyro is diagnosed", fr.gyro == "nworld" and fr.gyroScore > 0.8, tostring(fr.gyro) .. " " .. tostring(fr.gyroScore))
  local halG = HAL.sim(cfg, truth, res)
  local kicked = false
  local cyc = halG.cycle
  halG.cycle = function(a, b) if not kicked and halG.sim.t > 6 then kicked = true; halG.sim.w.x = halG.sim.w.x + 0.2; halG.sim.w.z = halG.sim.w.z - 0.15 end return cyc(a, b) end
  local fr2 = F.hover(halG, res, { height = 5, secs = 5, maxSeconds = 60 })
  check("normal gyro is diagnosed as world frame", fr2.gyro == "world" and fr2.gyroScore > 0.8, tostring(fr2.gyro) .. " " .. tostring(fr2.gyroScore))
end

local function testNav()
  local s, v, maxv, over = 0, 0, 0, false
  for _ = 1, 3000 do
    local a
    s, v, a = Nav.step(s, v, 30, 5, 1.5, 0.05)
    maxv = math.max(maxv, v)
    if s > 30 + 1e-9 then over = true end
    if s == 30 and v == 0 then break end
  end
  check("nav step reaches the goal without overshoot", s == 30 and not over and maxv <= 5 + 1e-9, string.format("s %.3f maxv %.2f", s, maxv))
  local s2, v2 = 0, 0
  for _ = 1, 3000 do s2, v2 = Nav.step(s2, v2, -12, 3, 1.0, 0.05) if s2 == -12 and v2 == 0 then break end end
  check("nav step works backwards", s2 == -12)
  local start = { x = 0, y = 60, z = 0 }
  local r = Nav.build({ { x = 5, z = 6 } }, start)
  check("nav default altitude is start + 3", r and r[1].y == 63 and r[1].speed == 5 and r[1].dwell == 2)
  check("nav rejects too high", Nav.build({ { x = 1, z = 1, y = 200 } }, start) == nil)
  check("nav rejects too low", Nav.build({ { x = 1, z = 1, y = 60.2 } }, start) == nil)
  check("nav rejects missing z", Nav.build({ { x = 1 } }, start) == nil)
  local cx, cz = Nav.compassVec(90)
  local nx, nz = Nav.compassVec(0)
  check("compass east is +x, north is -z", math.abs(cx - 1) < 1e-9 and math.abs(cz) < 1e-9 and math.abs(nx) < 1e-9 and math.abs(nz + 1) < 1e-9)
  local r2 = Nav.build({ { x = 5, z = 6, speed = 99 } }, start)
  check("nav clamps speed", r2[1].speed == 8)
end

local function testRoutes()
  for _, case in ipairs({ { "tri", {} }, { "quad+", { trans = true, kT = 8.3 } }, { "hex", { kT = 3.0 } } }) do
    local name, o = case[1], case[2]
    local hal, af, truth, cfg = simShip(name, o)
    local res = W.run(hal, af, { say = function() end, ask = function() return true end, nosave = true })
    if res then
      local halF = HAL.sim(cfg, truth, res)
      local s = halF.sense()
      local pts = { { x = s.p.x + 12, z = s.p.z, y = s.p.y + 3 }, { x = s.p.x + 12, z = s.p.z + 12 }, { x = s.p.x, z = s.p.z + 12, speed = 3 } }
      local fr = F.route(halF, res, pts, { home = true, maxSeconds = 400 })
      check(name .. " route flown to the end", fr.ok, tostring(fr.reason))
      check(name .. " all four waypoints reached", #fr.waypoints == 4, tostring(#fr.waypoints))
      local worst = 0
      for _, w in ipairs(fr.waypoints) do worst = math.max(worst, w.err) end
      check(name .. " waypoints reached within 0.6 m", worst < 0.6, tostring(worst))
      check(name .. " route tilt and lag small", fr.maxTilt < 10 and fr.maxLag < 2.5, string.format("tilt %.1f lag %.2f", fr.maxTilt, fr.maxLag))
      local fin = fr.final
      check(name .. " landed back at the start", fin and math.abs(fin.x - s.p.x) < 1.0 and math.abs(fin.z - s.p.z) < 1.0 and fin.y < s.p.y + 0.3,
        fin and string.format("%.2f %.2f %.2f", fin.x - s.p.x, fin.z - s.p.z, fin.y - s.p.y) or "nil")
    else
      check(name .. " wizard before route", false)
    end
  end
  -- a route that is too high is refused without flying
  local hal, af, truth, cfg = simShip("tri")
  local res = W.run(hal, af, { say = function() end, ask = function() return true end, nosave = true })
  local halF = HAL.sim(cfg, truth, res)
  local s = halF.sense()
  local fr = F.route(halF, res, { { x = s.p.x + 5, z = s.p.z, y = s.p.y + 500 } }, {})
  check("route refuses an absurd altitude", fr.ok == false and halF.sim.t == 0, tostring(fr.reason))
end

local function testTune()
  -- pure analysis on a synthetic delayed pulse response
  local ts, ws = {}, {}
  local alpha, Tp, delay, ratio = 0.10, 0.6, 0.25, 0.7
  for k = 0, 90 do
    local t = k * 0.02
    local u = math.max(0, math.min(t - 0.3 - delay, Tp))      -- pulse starts at 0.3, response delayed
    ts[#ts + 1], ws[#ws + 1] = t, ratio * alpha * u
  end
  local r, d = Tune.analyzeTorque(ts, ws, 0.3, Tp, alpha)
  check("analyzeTorque finds authority and delay", r and math.abs(r - 0.7) < 0.03 and math.abs(d - 0.25) < 0.03, tostring(r) .. " " .. tostring(d))
  local tv, vs = {}, {}
  for k = 0, 220 do local t = k * 0.02 tv[#tv + 1], vs[#vs + 1] = t, 0.55 * 1.2 * math.max(0, t - 0.4) end
  local g = Tune.analyzeLateral(tv, vs, 0.2, 1.6, 1.2)
  check("analyzeLateral finds the lateral gain", g and math.abs(g - 0.55) < 0.03, tostring(g))
  local zeros = {}
  for i = 1, #ts do zeros[i] = 0 end
  check("analyzeTorque rejects no response", Tune.analyzeTorque(ts, zeros, 0.3, Tp, alpha) == nil)

  -- full in-flight identification on a ship with weak torque, slow actuators and a weak +x side
  local over = { torqueScale = { x = 0.7, y = 0.6, z = 0.8 }, delay = 0.3, latScale = { xp = 0.6 } }
  local hal, af, truth, cfg = simShip("tri", { sim = over })
  local res = W.run(hal, af, { say = function() end, ask = function() return true end, nosave = true })
  local halT = HAL.sim(cfg, truth, res)
  local tune, why = Tune.run(halT, res, { trim = false, maxSeconds = 300 })
  check("tune completes", tune ~= nil, tostring(why))
  if tune then
    check("tune: torque authority", math.abs(tune.gx - 0.7) < 0.15 and math.abs(tune.gz - 0.8) < 0.15 and math.abs(tune.gy - 0.6) < 0.15,
      string.format("gx %.2f gz %.2f gy %.2f", tune.gx, tune.gz, tune.gy))
    check("tune: actuator delay", math.abs(tune.delay - 0.3) < 0.1, string.format("%.3f", tune.delay))
    check("tune: finds the weak +x side", math.abs(tune.lat.xp - 0.6) < 0.15 and math.abs(tune.lat.xn - 1) < 0.2 and math.abs(tune.lat.zp - 1) < 0.15 and math.abs(tune.lat.zn - 1) < 0.15,
      string.format("xp %.2f xn %.2f zp %.2f zn %.2f", tune.lat.xp, tune.lat.xn, tune.lat.zp, tune.lat.zn))
    res.tune = tune
    -- fly with and without the tune on the same ship: both safe, the tuned one tracks the weak side better
    local function leg(af2)
      local h = HAL.sim(cfg, truth, af2)
      local s0 = h.sense()
      return F.route(h, af2, { { x = s0.p.x + 14, z = s0.p.z, y = s0.p.y + 3, speed = 4 } }, { trim = false, maxSeconds = 300 })
    end
    local withT = leg(res)
    local plain = {}
    for k, v in pairs(res) do plain[k] = v end
    plain.tune = nil
    local without = leg(plain)
    check("tuned ship flies the route", withT.ok and #withT.waypoints == 1 and withT.waypoints[1].err < 0.6, tostring(withT.reason))
    check("untuned ship still arrives (lag cap holds the reference)", without.ok and #without.waypoints == 1, tostring(without.reason))
  end
end

local function testTrim()
  local path = "/ap_data/selftest_trim.json"
  pcall(fs.delete, path)
  local ge = { 0.85, 1.10 }
  local hal, af, truth, cfg = simShip("tri", { sim = {} })
  local res = W.run(hal, af, { say = function() end, ask = function() return true end, nosave = true })
  for j, r in ipairs(cfg.rotors) do r.gainErr = ge[j] or 1 end             -- the ship is not what the wizard measured
  local h1 = HAL.sim(cfg, truth, res)
  local f1 = F.hover(h1, res, { height = 3, secs = 25, trimPath = path, maxSeconds = 200 })
  check("trim flight 1 ok and saved", f1.ok and f1.trimSaved == true and not f1.trimLoaded, tostring(f1.reason))
  local h2 = HAL.sim(cfg, truth, res)
  local f2 = F.hover(h2, res, { height = 3, secs = 25, trimPath = path, maxSeconds = 200 })
  check("trim flight 2 loaded the trim", f2.ok and f2.trimLoaded == true, tostring(f2.reason))
  check("saved trim shortens the transient", f2.maxTilt < 0.6 * f1.maxTilt, string.format("tilt %.2f -> %.2f", f1.maxTilt, f2.maxTilt))
  -- a different mass invalidates it
  local heavy = {}
  for k, v in pairs(res) do heavy[k] = v end
  heavy.mass = res.mass * 1.2
  local A = require("airframe")
  check("trim ignored when the mass changed", A.loadTrim(heavy, path) == nil and A.loadTrim(res, path) ~= nil)
  pcall(fs.delete, path)
end

local function testTransit()
  -- pure pieces
  local cfg = { coneAccel = 1.9, dragK = 0.0002, dragBrake = 0, leanMax = 12, leanInflow = 2.0, leanDerate = 0.6, leanBrakeMax = 25, leanBrakeInflow = 14,
    gravity = 11, planAccelMin = 0.6, maxPlanAccel = 6, accel = 2, descendLead = 12, descendRadius = 140, descendMaxSpeed = 40 }
  local aFn, dFn = Nav.accelAt(cfg), Nav.decelAt(cfg)
  check("accel capability falls with speed (drag) and is bounded", aFn(0.5) >= aFn(60) and aFn(60) >= 0.6 and aFn(5) <= 6, string.format("%.2f %.2f", aFn(0.5), aFn(60)))
  check("braking capability exceeds the accelerating one at speed", dFn(25) > aFn(25), string.format("%.2f vs %.2f", dFn(25), aFn(25)))
  local T = Nav.stopTable(dFn, 40)
  check("stop table is monotonic and consistent", Nav.vForDist(T, T.d[#T.d] / 2) < 40 and Nav.vForDist(T, 0) == 0 and Nav.vForDist(T, 1e6) == T.v[#T.v])
  local s, v = 0, 0
  local pk = 0
  for _ = 1, 4000 do
    s, v = Nav.step(s, v, 400, 30, 2, 0.05, aFn, T, dFn)
    pk = math.max(pk, v)
    if s == 400 and v == 0 then break end
  end
  check("speed-dependent step arrives on the goal", s == 400 and pk <= 30 + 1e-9, string.format("s %.2f peak %.1f", s, pk))
  check("lean limit shrinks with speed", Nav.leanLimit(12, 5, 0.3, 0, nil, cfg, false) > Nav.leanLimit(12, 40, 0.3, 0, nil, cfg, false))
  check("lean limit is zero without motor headroom or with altitude sag", Nav.leanLimit(12, 5, 0.95, 0, nil, cfg, false) == 0 and Nav.leanLimit(12, 5, 0.3, 2.5, nil, cfg, false) == 0)
  check("lean limit is zero near the floor", Nav.leanLimit(12, 5, 0.3, 0, 1.0, cfg, false) == 0)
  check("braking allows more lean", Nav.leanLimit(25, 20, 0.3, 0, nil, cfg, true) > Nav.leanLimit(25, 20, 0.3, 0, nil, cfg, false))
  check("descent starts from the ship state", Nav.descentStart(30, 10, 250, 400, cfg) and not Nav.descentStart(300, 10, 250, 400, cfg) and not Nav.descentStart(30, 10, 100, 400, cfg))

  -- full transit flights in the simulator (drag and lift lost to inflow): no lean, then with lean
  for _, case in ipairs({ { "quad+", { trans = true, kT = 8.3, sim = { drag = 0.0005, inflowLoss = 0.08 } } }, { "tri", { sim = { drag = 0.0005, inflowLoss = 0.08 } } } }) do
    local name, o = case[1], case[2]
    local times = {}
    for _, lean in ipairs({ 0, 12 }) do
      local hal, af, truth, cfgS = simShip(name, o)
      local res = W.run(hal, af, { say = function() end, ask = function() return true end, nosave = true })
      local halF = HAL.sim(cfgS, truth, res)
      local s0 = halF.sense()
      local fr, info = F.transit(halF, res, { X = s0.p.x + 300, Z = s0.p.z + 40, vmax = 20, accel = 2, lean = lean, floor = 12, cruise = 16 }, { trim = false, maxSeconds = 400 })
      local tag = string.format("%s transit lean %d", name, lean)
      check(tag .. " flown to the end", fr.ok, tostring(fr.reason))
      local fin = fr.final
      local err = fin and math.sqrt((fin.x - (s0.p.x + 300)) ^ 2 + (fin.z - (s0.p.z + 40)) ^ 2) or 99
      check(tag .. " landed on the target", err < 2.0 and fr.landedAgl and fr.landedAgl < 0.5, string.format("err %.2f agl %s", err, tostring(fr.landedAgl)))
      check(tag .. " final approach stays clear of the ground", (fr.minDescentAgl or 0) > 1.2, tostring(fr.minDescentAgl))
      check(tag .. " reached cruise speed", (fr.peakSpeed or 0) > 15, tostring(fr.peakSpeed))
      check(tag .. " kept the floor", (fr.floorBreaches or 0) == 0 and (fr.minClear or 0) > -1.5, string.format("breach %s min %s", tostring(fr.floorBreaches), tostring(fr.minClear)))
      check(tag .. " tilt reasonable", fr.maxTilt < 22, string.format("%.1f", fr.maxTilt))
      times[lean] = fr.t
    end
    check(name .. " lean does not make the transit much slower", times[12] <= times[0] * 1.2 + 1, string.format("%.1f vs %.1f", times[12] or 0, times[0] or 0))
  end

  -- drag measurement: a few seconds of level coasting at speed gives the drag coefficient (the simulator's drag is 0.0005 per m)
  do
    local hal, af, truth, cfgS = simShip("quad+", { trans = true, kT = 8.3, sim = { drag = 0.0005, inflowLoss = 0.08 } })
    local res = W.run(hal, af, { say = function() end, ask = function() return true end, nosave = true })
    local halF = HAL.sim(cfgS, truth, res)
    local s0 = halF.sense()
    local fr = F.transit(halF, res, { X = s0.p.x + 800, Z = s0.p.z, vmax = 24, accel = 2, lean = 12, floor = 12, cruise = 16 },
      { trim = false, maxSeconds = 400, navOver = { coastSpeed = 20, coastSecs = 5 } })
    local c = fr.coast
    check("coast measurement flight completes", fr.ok and fr.waypoints[1] and fr.waypoints[1].err < 3, tostring(fr.reason))
    check("the coast measures the drag coefficient", c and c.k > 0.0003 and c.k < 0.0008, c and string.format("k %.2e from %.1f to %.1f m/s in %.1f s", c.k, c.v0, c.v1, c.t) or string.format("no result: peak %.1f m/s tilt %.1f t %.0f", fr.peakSpeed or 0, fr.maxTilt or 0, fr.t or 0))
  end
end

local function testNavAdapt()
  -- a point-mass "ship" that brakes at only 55 % of what the plan assumes: the follower must learn that and still stop at the goal
  local cfg = { plan = true, speed = 40, maxSpeed = 40, accel = 2, coneAccel = 1.9, dragK = 0.0002, leanMax = 12, gravity = 11, maxLag = 30,
    descendLead = 12, descendRadius = 140, descendMaxSpeed = 20, altTol = 5, arriveDist = 1.5, arriveSpeed = 0.8, climbRate = 20, vertAccel = 20, legStartY = 0 }
  local route = { { x = 600, z = 0, y = 10, speed = 40, dwell = 0.5 } }
  local R = Nav.new(route, { x = 0, y = 0, z = 0 }, cfg)
  local ship = { x = 0, y = 10, z = 0, vx = 0, vz = 0, vy = 0 }
  local dt, maxx = 0.05, 0
  local aF, dF = R.aFn, R.dFn
  for k = 1, 4000 do
    Nav.update(R, dt, ship, k * dt)
    -- the ship tracks the reference velocity with its own limits (accel as planned, braking at 55 %)
    local want = R.vx + 0.5 * (R.x - ship.x)
    local dv = want - ship.vx
    local lim = (dv >= 0) and aF(math.abs(ship.vx)) * dt or dF(math.abs(ship.vx)) * 0.55 * dt
    ship.vx = ship.vx + math.max(-lim, math.min(lim, dv))
    ship.x = ship.x + ship.vx * dt
    maxx = math.max(maxx, ship.x)
    if R.done then break end
  end
  check("follower learns the real braking capability", R.brakeScale < 0.7, string.format("%.2f", R.brakeScale))
  check("follower stops at the goal despite weak braking", R.done and maxx < 600 + 25, string.format("max x %.1f done %s scale %.2f cap %.2f", maxx, tostring(R.done), R.brakeScale, R.capScale))

  -- a transient (e.g. the change from accelerating to braking) can push the learned braking scale far down: while the ship then keeps up with
  -- the reference the scale must creep back toward the value the leg started with (and never above it)
  local function legScale(recover)
    local c2 = {}
    for k, v in pairs(cfg) do c2[k] = v end
    c2.brakeScale0, c2.brakeRecover, c2.dragBrake = 0.8, recover, 0.6
    c2.speed, c2.maxSpeed = 60, 60
    local rt = { { x = 4000, z = 0, y = 10, speed = 60, dwell = 0.5 } }
    local R2 = Nav.new(rt, { x = 0, y = 0, z = 0 }, c2)
    local sh = { x = 0, y = 10, z = 0, vx = 0, vz = 0, vy = 0 }
    local hit, top = false, 0
    for k = 1, 8000 do
      Nav.update(R2, dt, sh, k * dt)
      sh.vx = R2.vx
      sh.x = R2.x
      if not hit and R2.leg and R2.leg.v > 40 then hit = true R2.brakeScale = 0.4 end
      top = math.max(top, R2.brakeScale)
      if R2.done then break end
    end
    return R2.brakeScale, top
  end
  local sHold = legScale(0)
  local sRec, sTop = legScale(0.03)
  check("braking scale recovers while the ship keeps up, never above its start value", sRec > sHold + 0.15 and sTop <= 0.8 + 1e-9, string.format("scale %.2f -> %.2f (top %.2f)", sHold, sRec, sTop))

  -- a ship that can do more than the plan assumes: the plan probes upward and the leg gets faster; a ship that cannot still arrives on the goal
  local function legProbe(top, shipFactor)
    local c2 = {}
    for k, v in pairs(cfg) do c2[k] = v end
    c2.capScaleMax, c2.speed, c2.maxSpeed, c2.brakeScale0, c2.maxLag = top, 120, 120, 0.8, 30
    local rt = { { x = 6000, z = 0, y = 10, speed = 120, dwell = 0.5 } }
    local R2 = Nav.new(rt, { x = 0, y = 0, z = 0 }, c2)
    local sh = { x = 0, y = 10, z = 0, vx = 0, vz = 0, vy = 0 }
    local aF2, dF2 = R2.aFn, R2.dFn
    local pk, mx = 0, 0
    for k = 1, 12000 do
      Nav.update(R2, dt, sh, k * dt)
      local want = R2.vx + 0.5 * (R2.x - sh.x)
      local dv = want - sh.vx
      local lim = (dv >= 0) and aF2(math.abs(sh.vx)) * shipFactor * dt or dF2(math.abs(sh.vx)) * dt
      sh.vx = sh.vx + math.max(-lim, math.min(lim, dv))
      sh.x = sh.x + sh.vx * dt
      pk, mx = math.max(pk, sh.vx), math.max(mx, sh.x)
      if R2.done then return k * dt, pk, mx, R2.capScale end
    end
    return 600, pk, mx, R2.capScale
  end
  local tBase, pBase = legProbe(1, 1.6)
  local tProbe, pProbe, mProbe, cProbe = legProbe(1.8, 1.6)
  check("a capable ship is flown faster when the plan probes upward", tProbe < tBase - 3 and pProbe > pBase + 3, string.format("%.0f s %.0f m/s -> %.0f s %.0f m/s (scale %.2f)", tBase, pBase, tProbe, pProbe, cProbe))
  local _, _, mWeak, cWeak = legProbe(1.8, 0.8)
  check("a ship that cannot follow the probe still stops on the goal", mWeak < 6000 + 30, string.format("max x %.1f scale %.2f", mWeak, cWeak))
end

local function testRecover()
  local path = "/ap_data/selftest_lock.json"
  pcall(fs.delete, path)
  local hal, af, truth, cfg = simShip("quad+", { trans = true, kT = 8.3 })
  local res = W.run(hal, af, { say = function() end, ask = function() return true end, nosave = true })
  local r0 = F.recover(HAL.sim(cfg, truth, res), res, { lockPath = path })
  check("recover without a lock does nothing", r0.ok == false and r0.reason == "no flight lock")
  -- the computer restarted while the ship hung 18 m above the ground
  local lf = fs.open(path, "w")
  lf.write(textutils.serialiseJSON({ ground = 20 }))
  lf.close()
  local halF = HAL.sim(cfg, truth, res)
  halF.sim.p.y = 38
  local fr = F.recover(halF, res, { lockPath = path, maxSeconds = 300 })
  check("recover lands a ship found in mid-air", fr.ok and fr.landedAgl and fr.landedAgl < 0.5, tostring(fr.reason) .. " " .. tostring(fr.landedAgl))
  check("recover clears the lock", not fs.exists(path))
  check("recover keeps the ship near where it was", fr.final and math.abs(fr.final.x) < 3 and math.abs(fr.final.z) < 3, fr.final and string.format("%.1f %.1f", fr.final.x, fr.final.z) or "nil")
  -- on the ground already: only stop
  lf = fs.open(path, "w")
  lf.write(textutils.serialiseJSON({ ground = 20 }))
  lf.close()
  local r2 = F.recover(HAL.sim(cfg, truth, res), res, { lockPath = path })
  check("recover on the ground only stops the rotors", r2.ok and r2.reason == "already on the ground" and not fs.exists(path))
  pcall(fs.delete, path)
end

local function calibrated(layout, o)
  local hal, af, truth, cfg = simShip(layout, o)
  local res = W.run(hal, af, { say = function() end, ask = function() return true end, nosave = true })
  return res, truth, cfg
end

local function testParity()
  -- refresh: a heavier ship gets a higher transmission level, a moved centre of mass moves the arms
  local res = calibrated("quad+", { trans = true, kT = 8.3 })
  local lv0 = res.rotors[1].level
  local info = { mass = res.mass * 1.35, g = res.g, inertia = { x = res.inertia.x * 1.35, y = res.inertia.y * 1.35, z = res.inertia.z * 1.35 }, rotors = {} }
  for j, r in ipairs(res.rotors) do info.rotors[j] = { bearing = r.bearing, pos = { x = r.x + 0.5, y = r.y, z = r.z } } end
  local ok = AFm.refresh(res, info)
  check("refresh re-plans for a heavier ship", ok and res.rotors[1].level >= lv0 and res.rotors[1].level <= 15 and math.abs(res.mass - 80.5 * 1.35) < 0.5, tostring(lv0) .. " -> " .. tostring(res.rotors[1].level))
  check("refresh follows the centre of mass", math.abs(res.rotors[1].x - (info.rotors[1].pos.x)) < 1e-9)
  local tooHeavy = { mass = res.mass * 6, g = res.g, inertia = res.inertia, rotors = info.rotors }
  local ok2 = AFm.refresh(res, tooHeavy)
  check("refresh refuses a ship too heavy for its propellers", ok2 == false)

  -- facing the travel direction: a weak +x side is turned off the travel axis
  local weak = { xp = 0.4, xn = 1, zp = 1, zn = 1 }
  local tgt = Nav.bestHeading(weak, 1, 0, 0)
  check("bestHeading turns a weak side off the travel axis", tgt and math.abs(math.abs(tgt) - 90) <= 15, tostring(tgt))
  check("bestHeading leaves a good ship alone", Nav.bestHeading({ xp = 1, xn = 1, zp = 1, zn = 1 }, 1, 0, 0) == nil and Nav.bestHeading(weak, 0, 1, 0) == nil)
  local res4, truth4, cfg4 = calibrated("tri", { sim = { latScale = { xp = 0.4 } } })
  res4.tune = { lat = weak }
  local h4 = HAL.sim(cfg4, truth4, res4)
  local q0 = h4.sense()
  local fr4 = F.route(h4, res4, { { x = q0.p.x + 28, z = q0.p.z, y = q0.p.y + 3, speed = 3 } }, { trim = false, maxSeconds = 400 })
  check("route with a weak side turns first and arrives", fr4.ok and (fr4.turns or 0) == 1 and #fr4.waypoints == 1 and fr4.waypoints[1].err < 1.0, tostring(fr4.reason) .. " turns " .. tostring(fr4.turns))
  check("the ship ended turned about 90 deg", fr4.finalPsi and math.abs(math.abs(fr4.finalPsi) - 90) < 20, tostring(fr4.finalPsi))

  -- thin air: thrust per RPM falls with height; the controller scales the commands with the measured pressure
  local pr = function(y) return 1 - 0.0012 * (y - 20) end
  local r2, truth2, cfg2 = calibrated("tri", { sim = { pressure = pr } })
  local h2 = HAL.sim(cfg2, truth2, r2)
  local s0 = h2.sense()
  local fr = F.route(h2, r2, { { x = s0.p.x, z = s0.p.z, y = s0.p.y + 45, dwell = 12 } }, { trim = false, maxSeconds = 300 })
  check("climb to 45 m in thinning air ok", fr.ok, tostring(fr.reason))
  check("pressure scale was applied", fr.minPressureScale and fr.minPressureScale < 0.97 and fr.minPressureScale > 0.9, tostring(fr.minPressureScale))
  check("altitude held in thin air", fr.holdRms and fr.holdRms < 0.1, tostring(fr.holdRms))
end

local function testDiag()
  -- pushmap on a ship with a weak +x side
  local res, truth, cfg = calibrated("tri", { sim = { latScale = { xp = 0.6 } } })
  local out = Diag.pushmap(HAL.sim(cfg, truth, res), res, { trim = false, maxSeconds = 400 })
  check("pushmap flight ok", out.flight.ok, tostring(out.flight.reason))
  local g0, g180
  for _, r in ipairs(out.table) do if r.deg == 0 then g0 = r.gain elseif r.deg == 180 then g180 = r.gain end end
  check("pushmap finds the weak +x side", g0 and g180 and g0 < 0.75 and g180 > 0.85, string.format("%s %s", tostring(g0), tostring(g180)))
  check("pushmap reports the weak directions", #out.weak >= 1 and out.weak[1] <= 45, table.concat(out.weak, ","))

  local res2, truth2, cfg2 = calibrated("quad+", { trans = true, kT = 8.3 })
  local gu = Diag.gust(HAL.sim(cfg2, truth2, res2), res2, { trim = false, maxSeconds = 400 })
  check("gust flight ok", gu.flight.ok and #gu.tests == 4, tostring(gu.flight.reason))
  local worst = 0
  for _, t in ipairs(gu.tests) do worst = math.max(worst, t.settle or 99) end
  check("every gust is recovered from", worst < 8, tostring(worst))
  check("the lateral gust moves the ship but not far", gu.tests[1].maxDev > 0.1 and gu.tests[1].maxDev < 2.5, tostring(gu.tests[1].maxDev))

  local res3, truth3, cfg3 = calibrated("tri", {})
  local im = Diag.imbal(HAL.sim(cfg3, truth3, res3), res3, { maxSeconds = 600 })
  check("imbal flight ok", im.flight.ok, tostring(im.flight.reason))
  check("imbal: tilt and position stay small", im.peak.tilt < 15 and im.peak.dev < 4, string.format("tilt %.1f dev %.2f", im.peak.tilt, im.peak.dev))
  local t2 = im.times[1]
  local sn = im.learned[t2]
  check("imbal: the integrators found a hidden imbalance", sn and (math.abs(sn.ix) + math.abs(sn.iz) + math.abs(sn.ia)) > 0.01, sn and string.format("%.3f %.3f %.3f", sn.ix, sn.iz, sn.ia) or "nil")

  local ls = Diag.leanstep(HAL.sim(cfg2, truth2, res2), res2, { trim = false, maxSeconds = 400 })
  check("leanstep flight ok", ls.flight.ok and #ls.steps == 6, tostring(ls.flight.reason))
  local okEff = true
  for _, st in ipairs(ls.steps) do if not st.effect or st.effect < 0.6 or st.effect > 1.4 then okEff = false end end
  check("leanstep effectiveness near 1 at hover", okEff, ls.steps[1] and tostring(ls.steps[1].effect))

  local yc = Diag.yawcal(HAL.sim(cfg2, truth2, res2), res2, { trim = false, maxSeconds = 400 })
  check("yawcal covers every rotor with a sane ratio", yc.flight.ok and #yc.rotors == 4 and yc.rotors[1].ratio and yc.rotors[1].ratio > 0.3 and yc.rotors[1].ratio < 2.5,
    yc.rotors[1] and tostring(yc.rotors[1].ratio) or "nil")
end

local function testTandem()
  local cfg = Sim.config("tandem")
  local al, err = A.new(cfg.rotors)
  check("tandem allocator builds", al ~= nil, err)
  if not al then return end
  check("tandem: roll comes from the tilts, pitch from the thrust", al.rollTilt == true and al.pitchTilt == false and al.ax == nil and al.az ~= nil)
  local W0 = cfg.mass * cfg.g
  local Tt = A.thrust(al, W0, 0, 900)
  local fy, tzz = 0, 0
  for j, r in ipairs(cfg.rotors) do fy = fy + Tt[j]; tzz = tzz + r.x * Tt[j] end
  check("tandem thrust exact (lift and pitch)", near(fy, W0, 1e-6) and near(tzz, 900, 1e-6), fy .. " " .. tzz)
  local tl, ti = A.tilts(al, Tt, 120, nil, 400, 0.3, 250, nil)
  check("tandem tilts give force along x, yaw and roll", tl and near(ti.Fx, 120, 1e-6) and near(ti.tauY, 400, 1e-6) and near(ti.tauX, 250, 1e-6) and ti.scale == 1,
    tl and (ti.Fx .. " " .. ti.tauY .. " " .. ti.tauX) or "nil")
  check("a rotor pair level with the centre of mass is rejected", A.new({ { x = -5, y = 0, z = 0 }, { x = 5, y = 0, z = 0 } }) == nil)
  check("coaxial rotors at one point are rejected", A.new({ { x = 0, y = 2, z = 0 }, { x = 0, y = 3, z = 0 } }) == nil)

  -- closed loop on the simulated tandem (wizard first)
  local res, truth, cfgS = calibrated("tandem", { kT = 20 })
  check("tandem wizard completes", res ~= nil)
  if res then
    local h = HAL.sim(cfgS, truth, res)
    local fr = F.hover(h, res, { height = 4, secs = 14, maxSeconds = 200 })
    check("tandem hovers", fr.ok, tostring(fr.reason))
    check("tandem holds altitude and stays level", fr.holdRms and fr.holdRms < 0.15 and fr.maxTilt < 12, string.format("rms %s tilt %.1f", tostring(fr.holdRms), fr.maxTilt))
    -- the identification flight (gentle attitude gains, lean-only roll axis) must also stay stable
    local ht = HAL.sim(cfgS, truth, res)
    local tuneT, why = Tune.run(ht, res, { trim = false, maxSeconds = 400 })
    check("tandem tune flight completes", tuneT ~= nil, tostring(why))
    if tuneT then check("tandem tune finds the roll authority near 1", tuneT.gx > 0.6 and tuneT.gx < 1.6, string.format("gx %.2f gz %.2f gy %.2f", tuneT.gx, tuneT.gz, tuneT.gy)) end
    local h2 = HAL.sim(cfgS, truth, res)
    local s0 = h2.sense()
    local fr2 = F.route(h2, res, { { x = s0.p.x + 12, z = s0.p.z + 8, y = s0.p.y + 4, speed = 2 } }, { trim = false, maxSeconds = 400 })
    check("tandem flies a leg (lean-only roll axis)", fr2.ok and #fr2.waypoints == 1 and fr2.waypoints[1].err < 1.2, tostring(fr2.reason) .. " " .. (fr2.waypoints[1] and tostring(fr2.waypoints[1].err) or "-") .. string.format(" t %.0f tilt %.1f turns %s lvl %s lag %.1f drift %.1f", fr2.t or 0, fr2.maxTilt or 0, tostring(fr2.turns), tostring(fr2.levelModes), fr2.maxLag or 0, fr2.maxDrift or 0))

    -- a pure sideways step on the lean-only roll axis: settles without a big overshoot (the position integrator once wound up and parked the heli 4 m off)
    local h5 = HAL.sim(cfgS, truth, res)
    local s5 = h5.sense()
    local dbg5, dbgLast = {}, -9
    local fr5 = F.route(h5, res, { { x = s5.p.x, z = s5.p.z + 6, y = s5.p.y + 4, speed = 1.5, dwell = 6 } }, { trim = false, maxSeconds = 400,
      log = function(t, st, rpm, phase, info, ex)
        if t - dbgLast >= 2 then dbgLast = t; dbg5[#dbg5 + 1] = string.format("t%.0f z%.2f/%.2f vz%.2f tilt%.1f", t, st.pz - s5.p.z, ex.ref.pz - s5.p.z, st.vz, math.acos(math.min(1, st.cosTilt)) * 57.3) end
      end })
    if not (fr5.ok and fr5.waypoints[1]) then print(table.concat(dbg5, " | ")) end
    check("tandem sideways step settles on target", fr5.ok and fr5.waypoints[1] and fr5.waypoints[1].err < 1.0 and fr5.maxDrift < 7.5 and not fr5.arriveNote,
      tostring(fr5.reason) .. " " .. (fr5.waypoints[1] and tostring(fr5.waypoints[1].err) or "-") .. " drift " .. tostring(fr5.maxDrift) .. " " .. tostring(fr5.arriveNote))
    -- a fast lean transit on the tandem (pitch lean through saturating rotors): no wind-up, no runaway tilt, lands at the target
    local h4 = HAL.sim(cfgS, truth, res)
    local s4 = h4.sense()
    local fr4, ex4 = F.transit(h4, res, { X = s4.p.x + 320, Z = s4.p.z, cruise = 40, vmax = 35, accel = 2, floor = 20, lean = 20 }, { trim = false, maxSeconds = 400 })
    check("tandem lean transit 320 m completes", fr4.ok and fr4.waypoints[1] and fr4.waypoints[1].err < 3, tostring(fr4.reason) .. " " .. (fr4.waypoints[1] and tostring(fr4.waypoints[1].err) or "-"))
    check("tandem lean transit keeps the tilt moderate and never needs levelling", fr4.maxTilt < 28 and (fr4.levelModes or 0) == 0, string.format("tilt %.1f levelModes %s peak %.1f m/s", fr4.maxTilt, tostring(fr4.levelModes), fr4.peakSpeed or 0))

    -- nose-first flight: the front is the tail side here (-x), so a leg along +x makes the ship turn around first
    check("the wizard sets the default front (no zone words: +x) and flies tandems nose-first", res.front and res.front.axis == "+x" and res.front.nose == true, res.front and (res.front.axis .. tostring(res.front.nose)) or "nil")
    AFm.setFront(res, "-x", true, "manual")
    local h3 = HAL.sim(cfgS, truth, res)
    local s3 = h3.sense()
    local fr3 = F.route(h3, res, { { x = s3.p.x + 24, z = s3.p.z, y = s3.p.y + 4, speed = 3 } }, { trim = false, maxSeconds = 500 })
    check("nose-first: front -x toward the travel direction turns the ship about 180 deg", fr3.ok and (fr3.turns or 0) == 1 and fr3.waypoints[1] and fr3.waypoints[1].err < 1.5 and fr3.finalPsi and math.abs(fr3.finalPsi) > 160,
      tostring(fr3.reason) .. " turns " .. tostring(fr3.turns) .. " psi " .. tostring(fr3.finalPsi))
  end
end

local function testCruise()
  -- cruise height: thinner air lowers the drag but costs motor headroom; longer trips justify a higher cruise
  local pr = function(y) return math.max(0, 1 - 0.0035 * (y - 20)) end        -- like the real world: about -0.35 %/m
  local res, truth, cfg = calibrated("quad+", { trans = true, kT = 8.3, sim = { pressure = pr } })
  local h = HAL.sim(cfg, truth, res)
  local s = h.sense()
  local hShort, iS = F.bestCruise(h, res, s, 29, 150, { leanMax = 12, vmax = 100, dragK = 0.002 })
  local hLong, iL = F.bestCruise(h, res, s, 29, 30000, { leanMax = 12, vmax = 100, dragK = 0.002 })
  check("short trip keeps the low cruise height", hShort == 33, tostring(hShort))
  check("long trip cruises higher (thin air = less drag)", hLong > hShort + 20 and iL.v > iS.v, string.format("%s vs %s, %.1f vs %.1f m/s", tostring(hLong), tostring(hShort), iL.v or 0, iS.v or 0))
  local hn = F.bestCruise(HAL.sim(Sim.config("quad+"), truth, res), res, s, 29, 30000, { leanMax = 12 })
  check("without a pressure profile the cruise stays low", hn == 33, tostring(hn))
  -- lean slew limit from the torque capacity: the heavy tandem is slower than a light quad
  local resT = calibrated("tandem", { kT = 20 })
  local cT = Ctl.new(resT, {})
  local cQ = Ctl.new(res, {})
  check("lean slew limit follows the torque capacity", cT and cQ and cT.cfg.leanRateTransit < 20 and cT.cfg.leanRateTransit >= 3 and cQ.cfg.leanRateTransit >= cT.cfg.leanRateTransit, string.format("tandem %.1f quad %.1f deg/s", cT and cT.cfg.leanRateTransit or -1, cQ and cQ.cfg.leanRateTransit or -1))
end

local function testFront()
  local rot = { { zone = "aft", x = -11.1, z = 0 }, { zone = "fore", x = 5.9, z = 0 } }
  check("default front: fore is where the fore rotor is (+x)", AFm.defaultFront(rot) == "+x")
  check("default front: reversed ship (fore at -x)", AFm.defaultFront({ { zone = "aft", x = 6, z = 0 }, { zone = "fore", x = -9, z = 0 } }) == "-x")
  local q4 = { { zone = "fore_port", x = 6, y = 1.5, z = 6 }, { zone = "fore_starboard", x = -6, y = 1.5, z = 6 }, { zone = "aft_port", x = 6, y = 1.5, z = -6 }, { zone = "aft_starboard", x = -6, y = 1.5, z = -6 } }
  check("default front from a quad's zone words (+z)", AFm.defaultFront(q4) == "+z")
  check("default front without zone words is +x", AFm.defaultFront({ { zone = "rotor1", x = 3, z = 0 }, { zone = "rotor2", x = -3, z = 0 } }) == "+x")
  local af = { rotors = q4 }
  check("sides: aft is opposite the fore, port/starboard from the zone words", AFm.resolveSide(af, "aft") == "-z" and AFm.resolveSide(af, "port") == "+x" and AFm.resolveSide(af, "starboard") == "-x" and AFm.resolveSide(af, "-x") == "-x")
  check("an unknown side is refused", AFm.resolveSide(af, "up") == nil)
  check("noseHeading points the front along the travel direction", near(Nav.noseHeading({ off = 0 }, 0, 1), -90, 1e-6) and near(Nav.noseHeading({ off = -90 }, 1, 0), 90, 1e-6)
    and math.abs(Nav.noseHeading({ off = 180 }, 1, 0)) > 179.9, tostring(Nav.noseHeading({ off = 0 }, 0, 1)))
  local tl = { { zone = "aft", x = -11.1, y = 5, z = 0 }, { zone = "fore", x = 5.9, y = 3, z = 0 } }
  local afT = { rotors = tl }
  local fr = AFm.setFront(afT, "+x")
  check("a tandem defaults to nose-first, a quad does not", fr.nose == true and AFm.setFront({ rotors = q4 }, "+z").nose == false)
  check("front share counts the rotors on the front side", afT.front.share and afT.front.share > 0.45 and afT.front.share < 0.8, tostring(afT.front.share))
  check("front compass: heading of an unrotated ship points at its front axis", near(AFm.frontCompass({ rotors = tl, front = { axis = "-x" } }, { w = 1, x = 0, y = 0, z = 0 }), 270, 1e-6))
end

return function(only)
  F.testMode = true
  if only == "wizard" then testNav(); testWizard(); testFly(); testRoutes(); testTune(); testTrim()
  elseif only == "tune" then testTune(); testTrim()
  elseif only == "transit" then testTransit(); testNavAdapt(); testRecover()
  elseif only == "parity" then testParity(); testDiag()
  elseif only == "cruise" then testCruise()
  elseif only == "tandem" then testFront(); testTandem()
  else testMath(); testIds(); testAlloc(); testFlight(); testNav(); testWizard(); testFly(); testRoutes(); testTune(); testTrim(); testTransit(); testNavAdapt(); testRecover(); testParity(); testDiag(); testFront(); testTandem(); testCruise() end
  print(string.format("selftest: %d checks, %d failed", T.pass + T.fail, T.fail))
  return T.fail == 0
end
]=],
}
local args = { ... }
local names = {}
for p in pairs(FILES) do names[#names + 1] = p end
table.sort(names)

local function read(p) local f = fs.open(p, "r") if not f then return nil end local s = f.readAll() f.close() return s end

if args[1] == "check" then
  local bad = 0
  for _, p in ipairs(names) do
    if read("/" .. p) ~= FILES[p] then print("differs or missing: /" .. p) bad = bad + 1 end
  end
  print(bad == 0 and ("ap " .. VERSION .. ": all " .. #names .. " files match") or (bad .. " file(s) need updating: run apinstall"))
  return
end

print("Installing ap " .. VERSION)
for _, p in ipairs(names) do
  local f = fs.open("/" .. p, "w")
  if not f then error("cannot write /" .. p, 0) end
  f.write(FILES[p])
  f.close()
  if read("/" .. p) ~= FILES[p] then error("verify failed for /" .. p, 0) end
  print("  /" .. p)
end
local v = fs.open("/ap/VERSION", "w") v.write(VERSION) v.close()
-- run `ap recover` first thing at boot: if a flight was interrupted (chunk unload, crash) it lands the ship; otherwise it does nothing
local line = 'shell.run("/ap.lua", "recover")'
local st = read("/startup.lua") or ""
if not st:find('"recover"', 1, true) then
  local sf = fs.open("/startup.lua", "w")
  sf.write(line .. "\n" .. st)
  sf.close()
  print("added `ap recover` to /startup.lua (lands the ship after an interrupted flight)")
end
if not fs.exists("/ap_data") then fs.makeDir("/ap_data") end
print(#names .. " files installed. Your settings in /ap_data are kept.")
print("Next:  ap selftest     (checks the code, nothing flies)")
print("       ap scan         (finds the rotors on this ship)")
