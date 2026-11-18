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
