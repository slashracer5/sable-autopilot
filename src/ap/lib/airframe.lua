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
