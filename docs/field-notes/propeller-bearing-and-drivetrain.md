**Gyroscopic propeller bearing, redstone transmission, gearshift, creative motor, gimbal sensor: measured behaviour**

All values measured in game on a 4-bearing quadcopter ship (mass 56 to 67.5 as parts were added), Avionics 0.7.0, 2026-11-16. Where this contradicts the online docs or the method list above, **trust this section**; each item says how it was established.

### Drivetrain and thrust
- **Chain per prop**: `Create_CreativeMotor` -> `redstone_transmission` -> `create:gearshift` -> `gyroscopic_propeller_bearing`. Each prop had its own motor and transmission (no sharing).
- **`redstone_transmission`** (`createpropulsion:redstone_transmission`): `setShiftLevel(n)`, `getShiftLevel()`, `setTransmissionMode(m)`, `getTransmissionMode()`; mode read back as `"direct"`. Levels 0-15 are the usable range (the Avionics `analog_transmission` clamps to 0..15 and `redstone_transmission` behaves the same in use; the behaviour of out-of-range values was **not** characterised, do not probe it on an assembled propeller).
- **Bearing RPM = motor RPM x level / 15.** Level 1 with the motor at its default 256 RPM gives 17 RPM (`256/15`); level 2 with 225 motor RPM gave 29.9. `bearing.getSpeed()` returns that RPM. (`getRotationSpeed()` is deg/tick per the docs; the docs' eased-spin lag was not visible: commanded and actual bearing speed agreed within 0.12 RPM.)
- **Thrust is linear in bearing RPM: about 5.39 per bearing RPM** (91.6 / 183.4 / 275.2 at levels 1 / 2 / 3 with the 256 RPM motor), identical on all four bearings. Units are mod units (pN), compared with weight as `mass x 11` the hover estimate agreed (about 129-135 motor RPM per prop at level 4 for mass 60-67.5). Thrust responds with no measurable lag; the end-to-end **command -> ship response delay is a pure ~0.2 s**.
- **Control strategy that worked**: fix every transmission at one level as a range scaler (level 4: hover at about 130 of 256 motor RPM, ~2x headroom) and use `Create_CreativeMotor.setGeneratedSpeed(rpm)` as the continuous control input. The 16 transmission steps alone are far too coarse (one level is about half of hover thrust at level 2).
- **Restore state after every test**: transmissions to 0 and motors back to 256 (`getGeneratedSpeed()` returns the original). Never call `assemble()` / `disassemble()` on a bearing that carries a propeller.
- `gearshift` has no setter, only the kinetic read methods (`getSpeed`, `hasSource`, `getNetworkId`, ...). `ship_*` stress readings all stayed non-overstressed with creative motors.

### Handedness and the sign of thrust
- **`setThrustHandedness` takes exactly `"right_handed"` or `"left_handed"`** (anything else, e.g. `"left"`, errors with `expected 'right_handed' or 'left_handed'` and changes nothing).
- **Which handedness lifts depends on how the bearing is placed (owner's note).** On this build the bearings were placed **facing up with the motor/drive side below** (`getAxis()` = `up`, `getBlockNormal()` = `[0,1,0]`); there `right_handed` pushed the ship DOWN and **`left_handed` lifted** (confirmed by the user watching the airflow, then by flight). The mod's `getThrust()` sign was the opposite of the physical force: lift read **negative** (`-91.8` at 17 RPM left-handed, `+91.5` right-handed). Bearings placed in another orientation (facing down, sideways, drive side on top) were **not tested** and probably need the opposite setting: do not assume, and do not use `getThrust()` alone as a lift indicator. Check the placement (`getAxis`, `getBlockNormal`), then verify lift direction with a short low-power ground test and the user's eyes or an accelerometer reading.
- `getThrustVector()` stayed `[0,1,0]` in both cases; handedness flips the sign of thrust, not the direction vector.
- The change settles in about 3 s (read back after a delay).

### `setManualTarget` / `clearManualTarget` / gravity tracking (the big documentation error)
- **The online docs call `setManualTarget(target)` a "world-frame unit vector". It is not: it is in the SHIP-LOCAL (body) frame.** Established with a lean-step test: with the ship leaning, a manual target of `[0,1,0]` produced acceleration following the lean with gain 1.02 and only 0.3 deg of rotation error; sending a world-frame vector got rotated a second time by the ship's orientation, which cost ~85 % of lateral authority and made the force direction depend on heading. To steer thrust in world direction `d`, send `conj(orientation) * d` (rotate by the inverse of `sublevel.getLogicalPose().orientation`).
- **The 12 deg thrust-tilt cone is around local up** (the block normal; `getBlockNormal()` returned `[0,1,0]`). `getTiltAngle()` is in degrees in [0,12].
- **Default gravity tracking keeps thrust vertical**: a leaning ship gets no lateral force from it, which is why it is stabilising. Manual targets steer thrust directly: this is how a level ship can hold horizontal position (thrust vectoring) and yaw (tangential tilt).
- The target persists until `clearManualTarget()`: always clear it when your program exits or faults.
- `getManualTarget()` returns `nil` while gravity tracking. `getStabilizationStrength()` is 0 unpowered and 1 once spinning (at 0 the bearing snaps to the block normal and ignores targets).
- Setters (`setManualTarget`, `setThrustHandedness`, `setShiftLevel`, `setGeneratedSpeed`) are main-thread calls that yield to the next server tick.
- `getThrustVector()` reported world-frame values in near-level tests; it was not independently re-verified under large tilt.

### Yaw
- With all four props spinning the same way, the ship was **not yaw-stable**: heading drifted at -4.5 to +13 deg/s uncontrolled. Yaw hold worked by tilting every bearing's thrust tangentially (about +-4 deg gave about +-7 deg/s^2 yaw acceleration, ~1.85 deg/s^2 per degree, odd and clean) with the **sign measured, not derived** (this build needed yawSign -1 relative to its heading convention).

### Geometry: always verify physically
- The prop positions (which bearing is fore/starboard/port/aft in ship body axes) were first inferred from compass, yaw and the docs' "+X east, +Z south at identity" and were **wrong**, which made three flights diverge. The correct method: spin one prop alone at transmission level 1 on the ground and read the `gimbal_sensor` tilt direction. Result in this build: fore at body **-X**, aft +X, starboard **-Z**, port +Z.
- Body axes (`gimbal_sensor` frame = the sub-level, not the block): `getGravity()` ~ `(0,-11,0)` level. Because the controller's bow setting drives the zone names, a rebuild or wrench turn can change which name or axis a prop has: re-verify after any rebuild.

### `gimbal_sensor`
- `getAngles()` -> `{pitch, roll}` degrees; `getAngularRates()` -> `{wx, wy, wz}` deg/s; `getGravity()` body-frame m/s^2; `getLinearAcceleration()` proper acceleration (reads `-gravity` at rest, one tick of lag).
- Verified against pose-derived values: gravity vector identical to 1e-6; pitch equals the tilt of body +Z (positive = +Z end low), roll equals the tilt of body +X.
- **Rates are clean and lag-free**: `wx` follows d(tilt about Z)/dt (correlation 1.00), `wz` follows -d(tilt about X)/dt (-0.92), `wy` follows d(heading)/dt. Use them instead of differencing pose angles.
- It is a physical block: adding it raised ship mass by ~1 (59 -> 60). Read `sublevel.getMass()` at program start (hover RPM depends on it).
- Orientation of the mounted block does not matter; it reports in the ship's body frame.

### Ship centre of mass / trim
- The computer, controller, gimbal and modem make the ship slightly off-centre, but the dominant "imbalance" in the first flights was a wrong geometry sign, not mass: once fixed, the four motors needed only 131-137 RPM (very small trim).
