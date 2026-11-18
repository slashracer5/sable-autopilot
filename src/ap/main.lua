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
