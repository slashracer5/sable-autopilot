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
