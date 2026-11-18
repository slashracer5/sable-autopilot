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
