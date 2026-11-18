# General multirotor autopilot: design brief (started 2026-11-16)

Goal: take everything the original single-ship quadcopter controller (v1, not part of this repository) does
and make it work on any multirotor / VTOL ship with any rotor count and layout, set up by a wizard instead of by hand.

## Test beds
("Computer #N" in this log is the in-game ComputerCraft computer on test ship N; "the owner" is the repository owner, who flew the ships.)
- Computer #1: the quad (4 props, creative motor -> redstone transmission -> gyroscopic propeller bearing, mass ~80 t).
- Computer #2: tricopter (3 props: `aft`, `fore_port`, `fore_starboard`; 3 creative motors, NO redstone transmissions,
  4 wheel mounts, avionics controller, physics assembler; mass 36.5). All bearings left_handed, getAxis=up,
  idle thrust handedness already set. Motor speed is set directly with setGeneratedSpeed.
- Peripheral names derive from the bow (`fore_port`, `aft`), so the wizard must discover by prefix, not hard-code.

## Requirement (owner, 2026-11-16): everything runs inside ComputerCraft
Install, setup wizard, tuning, selftest, simulator, logging and flight all run on the in-game computer with no external
application. The PC side (an AI assistant driving the computer through a bridge, Python analysis) was only development tooling. Layout:
`ap.lua` launcher, `ap/main.lua` commands, `ap/lib/*` modules, data in `/ap_data`. `build.lua` (runs in CC) bundles
everything into one self-extracting `apinstall.lua` (`dist/`); `apinstall` installs/updates and keeps
`/ap_data`, `apinstall check` verifies. Tested: clean install on a spare computer, then `ap selftest` passes there.

## KEY DISCOVERY 2026-11-16: rotor positions come from the API
getSelfId / getNetworkId / getSourceId are the block position packed like BlockPos.asLong (x: top 26 bits, z: next 26,
y: low 12) in the ship's plot grid, the same frame as sublevel.getCenterOfMass(). Arm = blockCenter - CoM, no flight test.
Verified on both ships (quad: fore -9.7, aft +8.3 along x, port/starboard +-10 in z; tri: aft (0,-6.5), fore pair (+-4,+2.5)).
Motor<->bearing: bearing.getSourceId == motor.getNetworkId (direct drive) or same x/z column (with a transmission).
Transmissions carry no ids; paired by index and to be verified by a spin test in the wizard.

## Status
- Done and tested (88 selftest checks, all pass, inside CC on computers #0 and #2): math, hardware scan (`ap scan`),
  general allocator (any n >= 3 rotors), rigid-body simulator, physics-derived hover/position controller. Simulated on tri,
  quad+, quad-x, hex, octo and an asymmetric 5-rotor craft: climb, level recovery from 14 deg, yaw 60 deg, 12/-8 m position
  step, and robustness to inertia x1.4 with rotor gains off by -15 % / +10 %. Gains come from mass and inertia (a chosen
  bandwidth), not per-ship tuning.
- 2026-11-16 later: added `hal.lua` (same interface for the real ship and the simulator; `cycle` batches this step's
  writes with the next sample's reads, one tick), `airframe.lua` (/ap_data/airframe.json, levels, limits), `wizard.lua`
  (ground-only: motor->bearing and transmission->bearing pairing verified and repaired, every bearing made to push up,
  thrust per bearing RPM measured at two low speeds with lift capped near 25 % of the weight, level/limit planning),
  `fly.lua` (`ap fly [h] [s]`: ramp, climb, hold, controlled landing; tilt/drift/altitude/timeout faults; angular-rate sign
  check that cuts the flight if the gyro sign disagrees with the attitude change). 117 selftest checks pass in-game,
  including wizard repair of cross-wired motors/transmissions/handedness and sim hover flights for tri, quad and hex.
  NOT yet run on a real ship: `ap wizard` (spins rotors slowly) and `ap fly` need the owner's go-ahead per ship.
  Unverified on real hardware: getThrustVector direction vs handedness, sign/frame of sublevel.getAngularVelocity,
  no gimbal sensor on the tricopter (v1 used one on the quad).
- 2026-11-16 FIRST REAL FLIGHTS (tricopter #2, `ap fly 2 8`): flight 1 used the API angular velocity as a world vector and the
  ship oscillated with growing amplitude (tilt 19.5 deg); the safety check cut it, no damage. Cause: sublevel.getAngularVelocity is
  in the BODY frame (measured, correlation 1.00; world/negated hypotheses lose). Flight 2 used rates derived from the
  quaternion: climbed 2 m, held +-1 cm (rms 2.7 cm), max tilt 3.0 deg, drift 0.53 m, landed under control. Predicted hover
  RPM 70/90 vs actual 73/93. The HAL now rotates the API rate into the world frame; the controller still uses the derived rate.
  Landing note: the wheel suspension extends ~0.4 m when unloaded, so the 'stopped near ground' cut fires at ~0.4 m.
- 2026-11-16 QUAD (#1) with the SAME generic code: `ap wizard yes` -> kT 8.22 per bearing RPM (v1 calibration: 8.31), levels 4/3/4/4
  (v1 used 4), pairing verified; `ap fly 2 8 yes` -> hold rms 3.1 cm, max tilt 2.9 deg, drift 0.60 m, landed under control;
  the gyro check reports 'world' (the HAL now rotates the body-frame API rate). Log: `data/examples/quad_fly1.csv`. Both ships hover
  with no per-ship tuning, only the wizard's airframe.json.
- 2026-11-16 NAVIGATION PORTED (in-sim only so far): `nav.lua` (route validation, reference generator with speed/accel limits and
  80 % braking, lag cap, per-leg altitude, dwell, compass helper) and `fly.lua F.route` (ramp, vertical climb to the first
  waypoint altitude, legs, controlled landing at the last waypoint, contact-detect landing for uneven ground, drift/altitude/
  time faults). Commands: `ap goto X Z [Y] [speed]`, `ap move dx dz [h] [speed]` (out and back), `ap wp [file] [home]`,
  `ap cruise v heading secs`, `ap fly [h] [s]`; a trailing `yes` skips the 'area clear?' question. Altitude limits: waypoints
  1..60 m above the start. 143 selftest checks pass in-game (route flights on tri, quad+ and hex arrive within 0.6 m, tilt < 10 deg).
  NOT yet flown on a real ship. Not ported: transit/lean/floor, trim persistence, tuning commands, push/gust tests.
- 2026-11-16 TRIM + TUNE (installer 0.5.0-dev on #1/#2, 156 selftest checks pass): ctl.lua now has yaw and position integrators, `getTrim/setTrim`
  (saved to /ap_data/trim.json after a settled hover, reused for the same mass; a new wizard run invalidates it), per-axis torque
  authority (gx, gy, gz), actuator-delay-limited bandwidth, per-direction lateral authority. `ap tune` (tune.lua) identifies them in flight:
  frozen-loop torque pulses (~4 deg) + 1.4 s lateral pushes in the 4 body directions. REAL RESULTS: tri roll 0.64 pitch 0.72 yaw 0.99,
  delay 0.18 s, lateral 1.20/1.06/1.06/0.98 (no weak side); quad roll 0.43 pitch 0.42 yaw 0.94, delay 0.19 s (v1 measured ~0.2), lateral
  1.11/1.03/1.09/1.00. Flights with the tune: tri move 8 m max error 0.52 m, quad 0.50 m. `ap trim`, `ap showtune`, `ap cleartrim`.
  BUG FOUND on the way: the simulator's actuator delay had silently been zero (queued commands aliased the caller's arrays); fixed, all
  flights re-verified with the real 0.2 s delay.
- 2026-11-16 TRANSIT + RECOVERY (installer 0.7.0-dev; 192 selftest checks pass in-game): `ap transit X Z [cruise] [vmax] [accel] [floor] [lean]`:
  speed-dependent plan (cone + lean + drag), capability scale that drops when the ship lags, stop table from braking capability, ship-state
  limited speed, safety floor with breach braking, lean governor (inflow/rpm/sag/floor, latch), final-approach descent from the ship's state.
  REAL: tri 60 m at <= 10 m/s landed 0.14 m from the target, floor held, tilt 5.9 deg. A 300 m / 25 m/s run west LOST THE COMPUTER: the ship left
  the loaded chunks, the computer stopped mid-flight and had not reconnected (ship frozen in the air with its last motor commands).
  Countermeasures added: flight lock file (/ap_data/flight.lock, written at takeoff, removed after landing), `ap recover` (lands a ship found in
  mid-air using the recorded ground height; the installer puts it first in /startup.lua), and a range limit: routes farther than 100 m from the
  start are refused unless `range=N` is given (only do that when the whole route is loaded, e.g. follow the ship).
- 2026-11-16 CHUNK LOADERS: the owner added a chunk loader block to each ship (mass +1.0: tri 36.5->37.5, quad 80.5->81.5, still within the 5 %
  calibration tolerance). Verified: tri 220 m / 25 m/s transit and quad 219 m / 25 m/s transit both completed with the computer connected, landing
  0.22 m / 0.04 m off target (before, a 300 m run lost the computer). `ap recover` also proved itself: the quad's interrupted transit was landed at boot
  and its lock cleared. Bugs found in that recovery and fixed: the lock now stores the idle motor speeds (restoring the mid-flight ones left the
  motors running) and ground contact only counts below 1.5 m.
- 2026-11-16 LEAN AT SPEED on real ships (installer 0.8.0-dev, 192 checks): the lean governor measures sag from the peak altitude reached (a climbing ship has
  no sag) and transit uses a faster altitude integrator; a deadlock at the goal (ship past the goal before the reference) was fixed (min 0.5 m in the
  ship-based speed limit). REAL: tri 300 m, cruise 28 m, floor 16 m, lean 12: peak 24.7 m/s, up to 10.5 deg tilt, 2.8 m/s^2 acceleration (cone-only is ~1.9),
  max lag 16 m, landed 0.7 m off; quad 300 m, cruise 38 m, floor 29 m, lean 12: 54.7 s, tilt 10.8 deg, 0.28 m off. Data: see `flight-logs.zip` attached to the GitHub Release (a few examples in `data/examples/`).
  Remaining from v1: push/gust diagnostics, imbal test, yaw/heading commands, longer legs / top-speed runs (the chunk loaders now allow them).
- 2026-11-16 PARITY WITH v1 (installer 0.9.0-dev, 214 selftest checks): added dry, where, land, climb, calib, `key=value` overrides, mass/centre-of-mass/level refresh at every
  flight (v1 autoLevel+autoScale; the 5 % mass refusal is gone), air-pressure compensation (rpm / pressure ratio), faceTravel generalised (Nav.bestHeading from the
  tuned lateral authority; the ship turns before a leg so a weak side stays off the travel axis), attitude/altitude integrator boost when quasi-static (v1 attBoost/altBoost), and
  the diagnostics pushmap, gust, imbal, leanstep, yawcal, idpulse (diag.lua). REAL results: tri pushmap 0.95-1.10 in all 8 directions, quad 0.91-1.02: THE V1 'WEAK AFT SIDE'
  (29 % toward body +x) DOES NOT APPEAR with the allocator-based tilts: it was most likely an artefact of v1's tilt commands, not of the hardware. gust (tri/quad): all four
  disturbances recovered in 2.6-7.5 s; yawcal ratios 0.83-1.37 (tri), 1.05-1.26 (quad) of the physics prediction; leanstep: acceleration per unit of ACTUAL lean 0.96-0.99
  (a = g tan(tilt), no lift lost at hover) but the attitude loop only follows 55-68 % of a commanded lean within 2.5 s; imbal (tri, +12 %/-10 %/-8 %): peak tilt 9 deg, position
  error 3.4 m (7.3 m before the integrator boost), integrators found the imbalance. README.md documents install, setup, commands and safety.
- 2026-11-16 PERFORMANCE CHECK vs v1 (installer 1.0.0, 216 checks): quad 1000 m transit (cruise 33 m, floor 29 m, lean 12, vmax 60): peak 45.6 m/s (v1 44.6),
  76.9-77.6 s command-to-shutdown, landed 0.3 m off (v1: 64.3 s). Bugs found by these runs and fixed: (1) the stopping table was read at d/sb instead of d*sb
  (optimistic braking, 45-126 m overshoot -> drift fault); (2) the lean governor blocked braking lean during the final descent (sag measured from the cruise peak) and
  the descent started at 40 m/s: now sag is measured from the descent reference, the descent starts only below descendMaxSpeed (20 m/s) and when the CONE-ONLY stopping
  distance (learned braking scale) fits, and the plan brakes with lean above 20 m/s and with the cone below; (3) altitude integrator wound up at cruise could not unwind
  while |error| > 2 m (the descent stalled 2 m up); (4) the capability scale logic reacted to normal lean transients (thresholds now relative to the lag cap).
  The braking capability is learned in flight (ratio of achieved to planned deceleration, 0.92 on the quad) and stored in af.tune.planBrake for the next transit.
  Remaining gap to v1's best time: ~15 s of cone-only tail after the descent starts (v1 kept braking with lean longer); idea: start the descent so the ship
  reaches the 10 m lean-off height just as it is slow enough, and raise the braking-lean budget. Disk: each computer has ~1 MB: logs are capped (~1500 rows), `ap clean`.
- 2026-11-16 FRESH RUN ON A NEW SHIP TYPE: computer #3, a heavy TANDEM HELICOPTER (mass 537, two bearings 17 m apart on the x axis, 3-5 m above the centre of mass,
  inertia 5547 / 35709 / 36127, direct drive, no transmissions, no chunk loader known). Issues found and fixed, in order:
  1. scan refused it ("need at least 3 rotors") and alloc.new needed rank 3. The allocator is now rank-aware: a torque row exists only when the rotors are spread along its axis;
     a tandem (z = 0) gets ROLL from the bearing tilts (tau_x = sum y T tz), a side-by-side pair would get pitch the same way; layouts with no lever arm (level pair, coaxial at
     one point) are rejected with a reason. Tilt allocation rows are chosen to match (Fx or pitch, Fz or roll, yaw).
  2. translation on a lean-only axis (body z of the tandem) can only come from leaning: ctl.lua splits the position command per BODY axis, the lean-only part goes to the lean
     path with slow gains tied to the attitude bandwidth (leanOuterRatio * wnAtt), the bearing force request for that axis is dropped.
  3. the gentle identification flight (wnAtt 0.7) made the lean-only position loop oscillate (z +-5 m, drift fault): tune now uses a quicker attitude loop and a 30 m drift
     limit for lean-only ships; the simulator got a "tandem" layout and tests that reproduce this (alloc exactness, hover, leg, tune flight).
  4. pushmap showed ~0 gain along the lean-only axis because a 2 s push is shorter than the lean build-up: pushes are 4.5 s / 0.6 m/s^2 there.
  5. `ap tune` now logs to /ap_data/tune.csv (it failed silently before).
  Real results on the heli: wizard kT 28.1 per RPM (hover RPM 72 aft / 138 fore, as predicted 72/138), bearing cone measured 11.7 deg (setManualTarget doublets), hover hold rms 0.22 m
  untuned -> 0.018 m after `ap tune` (roll 1.05, pitch 0.73, yaw 0.95, delay 0.12 s, lateral +x 1.0/-x 1.23, +z 0.42/-z 0.67), move 25 m out/back 0.6/0.12 m, gust all recovered,
  goto 40 m sideways: the ship TURNED to heading 88 deg to put the travel on its strong axis and arrived 0.15 m off, transit 90 m at 15 m/s landed 0.1 m off, lean transit 81 m at 25 m/s
  turned to face travel and landed within 1.6 m. Wheels free-roll: a 30-degree bearing test with the rotors spinning moved the parked ship 18 m. 228 selftest checks, installer 1.1.0
  on #1/#2/#3.
- Next (old list): airframe.json writer + wizard (verify motor/transmission pairing by spin test, handedness, thrust calibration,
  authority check against the physics prediction), hardware actuation layer (batched setters), first tether-free hover on
  the tricopter, then port transit/landing/floor/lean/waypoints from v1.

## Architecture (proposal)
1. `airframe.json` per ship: rotors[] = {motor, bearing, transmission?, armBody (x,y,z from CoM), spin (+1/-1),
   thrustGain (N per RPM), rpmMax, coneDeg}, plus mass/inertia (read live) and limits. Written by the wizard.
2. Allocator: matrix B (collective + roll/pitch/yaw torque -> n rotor thrusts), least-squares with saturation handling
   (attitude has priority over collective, as in quad v1). Works for n >= 3 (tricopter: yaw via bearing tilt or
   differential, to be tested), including non-symmetric layouts.
3. Rotor identification (wizard): pulse each rotor, measure angular + linear acceleration, solve arm, gain and yaw
   sign. Reuse the v1 `idpulse`, `yawcal`, `calib`, `imbal` experience.
4. Carry over from v1 unchanged in spirit: mass/inertia scaling, trim integrators, position loop via thrust vectoring,
   trajectory generator, transit, controlled landing, faults, safety floor, lean governor (per-airframe parameters).
5. Pure functions with selftests, plus a simulated multirotor (tri, quad, hex, octo, offset) so the allocator and
   controller run before anything flies.
6. Wizard: discover ship, detect rotors, handedness check, direction tests, identification, first hover with ramp,
   tune, save airframe.json.

## Open questions (from the early design phase)
- Is the tricopter's yaw meant to come from bearing tilt, from motor-speed differences, or from reaction torque?
- Do you want one program (`ap.lua` + `airframe.json`) or a small library plus a front-end?


## 2026-11-17: front of the ship
* `af.front = {axis, nose, src, share}` (airframe.lua). Default axis = direction from the aft-zone rotors to the fore-zone rotors (the zone words come from Sable relative to the avionics controller; there is no controller-facing API: avionics_controller only has setTransponder/isTransponderOn). Rotor names without fore/aft words default to body +x.
* `nose` defaults to true for elongated layouts (extent along the front axis >= 1.8 x across, e.g. the tandem), false otherwise; the wizard asks for the side, `ap front <side> [nose|nonose]` changes it later without re-running the wizard.
* fly.lua onLegStart: legs >= 15 m turn the ship so the front points along the travel (Nav.noseHeading). If `ap tune` lateral authority says that heading is < 70 % as good as the best one, the weak-side rule wins (the front gives way, as when weight distribution demands it).
* Real tandem (#3) `ap move 30 0 5` nose-first: turned to heading 0, flew east, turned 180 deg, came back; 0.59 m / 0.15 m off, but 74 s instead of 31 s and 3.9 m peak drift during the turns (yaw slew is slow on 537 kg).
* Selftest 240 checks pass.


## 2026-11-17: long-haul / altitude work on the tandem (installer 1.3.0, 246 selftest checks)
* Measured on the real tandem (#3, chunk loader present): aero.getAirPressure(vec) falls steeply with height (1.29 at y=0, 0.86 at 100, 0.58 at 200, 0.31 at 300, 0 at 400);
  aero.getUniversalDrag exists (0.01 at the ship vs 0.09 default). Drag scales with the pressure, the rotors need rpm ~ 1/pressure.
  Runs (start y 67): 350 m / lean 20 / cruise 30: peak 19 m/s (planner capped by coneAccel = 1.9 x min lateral authority 0.42, i.e. the WEAK side, although a nose-first ship
  travels along its strong axis); 600 m / lean 18 / leanInflow 8: peak 40 m/s in 47 s; 2475 m / cruise 80 m / leanInflow 8: peak 69.3 m/s (still accelerating 1 m/s^2 at 68 m/s
  with tilt < 3 deg: the cone alone beat the drag), 81 s total, landed 0.27 m off. So the old plan (low cruise, inflow 2, weak-side cone) was far below the ship's capability.
* Planner changes: coneAccel from the lateral authority along the travel direction at the heading that will actually be flown (F.pickHeading / F.headingModes);
  F.bestCruise picks the cruise height from the real pressure profile (hal.pressureAt) by predicting climb + cruise + descent time with drag ~ pressure and the lean the motors can afford;
  dragK is scaled by the pressure at cruise; per-ship `ap set leanInflow|dragK|planBrake`; faster descent from altitude; leg starts near the cruise height (0.85 x) so the ship is not still
  climbing with saturated rotors when the reference starts to run (a drift fault at 44 m lag).
* BUG: the lean governor measured sag from the peak altitude: a climb that overshot the cruise height by 2 m switched the lean off for the whole flight (fixed: peak is capped at the altitude reference + 0.3 m).
* INCIDENT 2026-11-17 (6000 m / cruise 100 / leanInflow 20 test, before the fixes): the climb saturated the fore rotor (rpm 250), the reference outran the ship (lag 44 m = drift fault) and
  the ship went to 'land' at 12 m/s; the lean-only pitch loop wound up its integrator against the saturated rotors, overshot to 25-30 deg tilt and swung +-250 m around the landing point
  for 160 s (limit cycle, 47 m/s), then the tilt cut at 35 deg dropped it 6 m from 75 m: the heli ended on its side (tilt 68 deg). Fixes: attitude integrator anti-windup
  (frozen while the allocation is saturated), lean slew limit for transits from the torque capacity (Ctl.torqueCapacity -> leanRateTransit; position-hold lean keeps the fast slew: a slow slew
  inside the position loop made the tandem hold oscillate), the lean governor sees a 0.6 s average of the rpm demand, LEVEL MODE (above 75 % of maxTilt the position/lean demands are dropped and the
  ship is levelled; motors are cut only above 60 deg or within 2 m of the ground), maxTilt 45 for transits, faster descent when high. Data: see `flight-logs.zip` attached to the GitHub Release (a few examples in `data/examples/`).
* Estimated max speed of the heli: terminal speed with the cone alone ~ sqrt(2.28 / 0.00028) = 90 m/s at cruise 80 m; with lean far higher; NOT yet measured (needs the fixed controller flown).
* Re-test after the fixes (installer 1.3.0, leanInflow 8, cruise 80 m, lean 18, chunk loader present): 600 m auto-cruise 56 s; 2500 m peak 72.8 m/s in 91 s; 6062 m peak 102.7 m/s in 129 s (landed 0.15 m off);
  10 000 m peak 109.3 m/s = 393 km/h, still ~1 m/s^2 of acceleration left, 194 s to the target. Data: see `flight-logs.zip` attached to the GitHub Release (a few examples in `data/examples/`).
  Fit of the acceleration (a = a0 - k v^2, cone only, tilt < 3 deg, 40..97 m/s): a0 2.0-2.3, k 0.9-1.9e-4 -> terminal speed ~110-150 m/s at 80 m above the start (pressure ~0.75). 109 m/s is 73-99 % of that, so the
  80 % criterion is not demonstrated (the estimate has a wide range). The run was limited by the braking plan: the ship coasted/braked with the cone only (tilt 0.1 deg!) for ~100 s from 108 m/s
  (decel ~0.9 m/s^2); braking with whole-ship lean (leanBrakeInflow / leanBrakeMax) is the next gain.
* BUG found by that 10 km run: the ship reached the goal but sat 4.2 m off to the side, 1 m above the ground for 130 s until the time limit (the owner saw the heli hovering). The lean-only roll axis had a position
  integrator (kiPos 0.02, gate 3 m) FASTER than its own loop (wo = 0.24 rad/s): +-3.5 m swings with a 20 s period and a wound-up integral that cancelled the proportional lean. Fixes: lean-only integrator x0.1 and
  max 0.15 (cfg.leanOnlyKi / leanOnlyIntMax), velocity damping x1.5 (leanOnlyKd), plain-route reference accel 0.4 m/s^2 along a lean-only axis, and an arrival fallback (the leg ended, ship still < 0.5 m/s within 15 m
  for 12 s -> dwell/land, with a note). Real heli goto 10 m sideways: 0.6 m off (was a drift fault). Selftest 247 checks.


## 2026-11-17: braking and top speed (installer 1.5.0, 252 selftest checks)
Question: "improve the braking so the runs get faster". Findings from the 10 km log (heli_transit10000_cruise80.csv): the follower braked at 1.0-1.7 m/s2 with the tilt
at 0.1 deg for ~100 s. Two causes: (1) the learned braking scale (`brakeScale`) collapsed during the accelerate-to-brake transition (the ship is briefly above the reference) and never
came back, (2) the plan ignored that drag helps the braking.
Changes: braking is learned only after the ship has been clearly slower than the reference for 0.8 s; while the ship keeps up with the reference the scale creeps back
(0.03/s, never above the value the leg started with); the transit plan counts 60 % of the drag as braking help (`dragBrake` 0.6). The acceleration plan may now probe up to 1.8x the modelled
capability while the ship keeps up (`capScaleMax`, +0.03/s; braking keeps its own scale, so a probing plan never brakes harder than measured).
Drag measurement (`coastSpeed` / `coastSecs` nav settings): coast level for a few seconds at speed, k = (1/v1 - 1/v0)/t, stored as `dragK` (ground equivalent = k / pressure ratio).
Heli results (cruise 80 m above the start = ~147 m, air 73 %, leanInflow 8, lean 20):
- 12 km run (1.4.0, braking only): peak 125.9 m/s, braked 125 -> 17 m/s in 46 s (was ~100 s from 108 m/s), 191 s total (10 km took 194 s before).
- 14.9 km (1.4.1, probing): peak 141.5 m/s (509 km/h), 209 s.
- measured drag: 119.9 -> 115.3 m/s in 4.6 s => k 7.18e-5 per m at 73 % air (9.9e-5 at the ground; the old default guess 2e-4 was twice too high).
  With ~2.2 m/s2 of lateral thrust (cone 1.9 + a little lean) the top speed is sqrt(2.2/7.2e-5) = 175 m/s (163 m/s cone only).
- 14.9 km (1.5.0, measured dragK stored): peak 142.9 m/s (514 km/h), 210 s, 0.41 m off. That is 82 % of 175 m/s (88 % of 163 m/s). Acceleration at 140 m/s was ~0.75 m/s2 = 2.2 - 7.2e-5*140^2, so the ship is at its thrust limit and the estimate is consistent.
- A coast run (1.5.0, coastSpeed 120) peaked at only 122 m/s: after the coast the acceleration probe has to climb back from a low scale. Measure once, then fly without it.
Remaining gap to the true top speed is the ramp-up (reference accel at v > 100 m/s is < 1 m/s2): a leg much longer than 15 km would get closer to 175 m/s; whole-ship lean at speed is limited by the inflow budget (leanInflow 8 -> 3 deg at 140 m/s).

### Other ships with 1.5.x (2026-11-17)
- Measured drag (coast, stored as dragK, ground-pressure equivalent): heli 9.9e-5, quad 2.58e-4 (37 -> 35 m/s in 7 s), tri 1.93e-4 (30.8 -> 30.1 m/s in 4 s). The old default 2e-4 was right for the light ships and 2x too high for the heli.
- Quad 1 km transit (vmax 45): 66.8 s with a coast in it, 69.3 s for 943 m without (1.0.0: 77 s). Tri 500 m: 48.3 s.
- A coast at 30 m/s on the quad was unusable (the slow-down is below the noise): coast at >= 37 m/s for >= 7 s. The coast now re-anchors the reference to the ship every step (otherwise the reference ran 45 m ahead and the drift fault fired).
- The acceleration probe now backs off when lag appears and remembers 90 % of the scale where it did (capCeil). `capScaleMax=1` on the command line turns the probe off.
- (Fixed later the same day by `finalHeight`, see "Final approach fix" at the end.) OBSERVED: the quad's lag to the reference reaches ~45 m (the 45 m drift limit = maxLag 30 + 15) in the final approach in old logs too (quad_transit4/7_1km): the descent starts at ~16 m/s and the ship reaches the ground height while still moving at ~14 m/s, then the reference runs on. Worth a look at the descent start rule.

### Final approach fix (installer 1.5.3, 256 selftest checks)
Cause found in quad_transit943_probe.csv: the transit descent started 153 m out at 17 m/s and fell at ~6.6 m/s toward a target 1 m above the start height; the ship undershot it by ~1 m, touched the ground at 14 m/s (speed 14 -> 1 m/s in 0.6 s), crawled, then re-accelerated to the goal while the reference ran on (lag 45 m = the drift limit).
Fix: the transit's approach target is now start height + max(2.5 m, 0.15 x ship extent) (`finalHeight`), the landing phase takes the rest at 0.6 m/s; `res.minDescentAgl` is logged and selftested (> 1.2 m).
Results: quad 1 km 67.1 s (arrival at 57.4 s, was 69.3 s for 943 m), lowest approach height 1.4 m above ground, lag peak 31.8 m (was 44.9). Tri 500 m: arrival 41.0 s (48.3 s before), lag 31.8 (40.3). Heli 3 km: 105 s, 0.36 m off, lag 36.8.
