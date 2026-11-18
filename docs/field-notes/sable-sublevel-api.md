**`sublevel`, `aero`, `quaternion`, `matrix`: what the calls actually return (CC: Sable 1.3.4, measured in game)**

The help pages above list names only. Verified shapes and gotchas:

| Call | Returns / behaviour |
|---|---|
| `sublevel.isInPlotGrid()` | `true` when the computer is on an assembled Sable ship; cheap way to check the ship exists |
| `sublevel.getLogicalPose()` / `getLastPose()` | table `{ position = {x,y,z}, orientation = {a, v = {x,y,z}}, rotationPoint = {x,y,z}, scale = {x,y,z} }`. `position.y` is **world altitude**; `orientation` is a unit quaternion with scalar `a` and vector part `v` (note: `a`/`v`, not `w`/`x`/`y`/`z`); convention **body -> world** (verified against the gimbal sensor). `getLastPose` returned the identical table while the ship was still. Docs say both error when the computer is not on a ship (check `isInPlotGrid()` first). Which sample is newer/smoother was not characterised |
| `sublevel.getCenterOfMass()` and `pose.rotationPoint` | **plot-grid coordinates** (about `20481031, 129, 20481032`), not world coordinates. Only differences are meaningful |
| `sublevel.getMass()` | number; grows as blocks are added (56 bare ship, 59-60 with more devices, 67.5 for the finished quadcopter). Read at start, do not hard-code |
| `sublevel.getLinearVelocity()` / `getAngularVelocity()` / `getVelocity()` | `{x,y,z}`; angular velocity is rad/s. All are ~1e-6 at rest, not exact 0 |
| `sublevel.getInertiaTensor()` | a `matrix` object (3x3). Use **`I[r][c]`** (1-based). It has no `get`, `toArray` or `to2DArray`; the method set is `add, sub, mul, div, pow, inverse, determinant, transpose, trace, ...`. Quadcopter body-frame diagonal: 2239 (X), 3735 (Y, yaw), 1799 (Z) |
| `sublevel.getName()` | returns nothing (nil) until `setName` is used |
| `aero.getGravity()` | `{x=0,y=-11,z=0}` (**11, not 9.81**) |
| `aero.getAirPressure(pos)` | **requires a position argument** (a vector/table); calling with no argument errors `bad argument #1 (table expected, got nil)` |
| `aero.getUniversalDrag()` | 0.01 (no pressure dependence seen near sea level in the flat test world), `aero.getMagneticNorth()` was `{0,0,0}` there |
| `quaternion.fromShip()` | errors: "This method is no longer required as of CC: VS 0.4.0, as the Ship API now provides quaternions directly." Use `sublevel.getLogicalPose().orientation` |

**Cost**: every `sublevel.*` call waits one server tick (~50 ms; measured 20 calls = 1000 ms). Read the ones you need together with `parallel.waitForAll` (4 reads: 178 ms sequential, 49 ms parallel).

**JSON caveats** (matters when you `textutils.serialiseJSON` results, as a bridge that returns JSON does):
- Vectors (`{x,y,z}` tables) serialise fine.
- A `matrix` serialises as only `{"rows":3,"columns":3}`; the cells are lost. Read the cells yourself (`I[1][1]`...) before serialising.
- Do not call a getter you have not inspected with `pcall`; error text for a missing argument is returned as the failure string.

**Rotation usage that works**: rotate a body-frame vector to world with the pose quaternion `q` (`v_world = q * v_body * q^-1`), and world-to-body with the conjugate. Tilt readings from this agree with the `gimbal_sensor`.

**Units seen**: gravity 11 m/s^2, thrust in mod units (~5.39 per bearing RPM on the test build), velocity in m/s.
