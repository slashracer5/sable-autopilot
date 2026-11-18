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
