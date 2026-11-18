# sable-autopilot (`ap`): general multirotor autopilot for Create Aeronautics / Sable ships

A ComputerCraft program that flies **any** multirotor or VTOL ship built with Create Aeronautics / Sable propeller bearings. A setup wizard discovers the rotors, measures the thrust and writes an airframe file, and the same controller then hovers, flies routes and does fast long-distance transits on a tricopter, a quadcopter and a tandem helicopter with no per-ship tuning. Everything (install, setup, tuning, simulation, flight) runs on the in-game computer, no external tools.

Requires (versions it was developed and measured on): Minecraft 1.21.1 (NeoForge), CC: Tweaked, CC: Sable 1.3.4, Create Aeronautics 1.3.2, Create: Avionics 0.7.0. Peripheral and API facts that were measured in game are in [`docs/field-notes/`](docs/field-notes/).

> **Safety:** `ap wizard` spins rotors and `ap fly` / `ap transit` move a real (in-game) ship. Fly over open, loaded terrain with a chunk loader on the ship. The software has fault cut-outs and crash recovery, but it is provided as is (see [LICENSE](LICENSE)).

## Supported ships
Works on any ship with 2 or more upward-facing gyroscopic propeller bearings (3+ spread both ways, or a tandem / side-by-side pair that is above the centre of mass), each driven by a Create creative motor (optionally through a
redstone transmission).

## Install (once per computer)
1. The computer must be on the ship, with a wired modem cable run to the avionics controller so the `ship_*` peripherals are visible.
2. Put [`dist/apinstall.lua`](dist/apinstall.lua) on the computer (disk, paste, or `wget https://raw.githubusercontent.com/slashracer5/sable-autopilot/main/dist/apinstall.lua apinstall.lua` once the repo is public) and run `apinstall`. It installs `/ap.lua`, `/ap/…`, and puts
   `ap recover` first in `/startup.lua`. `apinstall check` verifies an installed copy. Your data in `/ap_data` is kept on updates.
3. `ap selftest` runs the built-in simulator checks (about one minute, nothing moves).

## Setup (the wizard)
1. `ap scan` lists the rotors and their positions (read from the block-position ids, no flight needed).
2. `ap wizard` (ground only, slow spins; the ship cannot lift off) verifies which motor/transmission drives which bearing and repairs wrong
   pairings, makes every bearing push up, measures thrust per RPM, plans transmission levels, and saves `/ap_data/airframe.json`.
   The wizard also asks which side is the FRONT of the ship (Enter keeps the default: the **fore** side of the avionics controller, found from the
   fore/aft words in the rotor names). Elongated ships such as tandem helicopters then fly every long leg nose-first.
3. `ap tune` (flies to 3 m, about a minute) measures torque authority, actuator delay and lateral authority per direction.
4. Optional: `ap trim 20` learns the ship's hidden imbalances and stores them (reused while the mass matches within 5 %).
Re-run the wizard after changing propellers or the layout. Adding or removing payload only needs `ap` to run: mass, inertia and arms are
refreshed from the ship at every flight and the transmission levels are re-planned.

## Flying
| command | what |
|---|---|
| `ap fly [h] [s]` / `ap land` | take off, hold h metres for s seconds, land |
| `ap goto X Z [Y] [speed]` | fly to world coordinates (Y absolute), land there |
| `ap move dx dz [h] [speed]` | out and back |
| `ap wp [file] [home]` | waypoint file `{"speed":5,"dwell":2,"home":true,"points":[{"x":..,"z":..,"y":..,"speed":..,"dwell":..}]}` |
| `ap cruise v heading secs [h]` | hold a speed on a compass heading (0 north, 90 east) |
| `ap climb peak [secs]` | climb, dwell, come down, land |
| `ap transit X Z [cruise] [vmax] [accel] [floor] [lean]` | fast flight above a safety floor, optional whole-ship lean (degrees); `-` for cruise / floor = automatic (cruise height from the air pressure profile) |
| `ap set [leanInflow\|dragK\|planBrake] [value\|default]` | per-ship planning values (leanInflow: how much lean the ship tolerates at speed, default 2; heavy rotors tolerate more) |
| `ap coastSpeed=120 coastSecs=5 transit ...` | drag measurement during a transit: once the ground speed passes `coastSpeed` the ship coasts level for a few seconds, the slow-down gives the drag coefficient, which is stored as `dragK` (ground-pressure equivalent) and printed together with the top speed it implies. The next flights plan with the measured drag. A measurement costs about 10-15 s of the flight (the acceleration plan has to find its way back up). |
| `ap dry [secs]` | read the sensors, print what would be commanded, never actuates |
| `ap where` | position, velocity, heading |
| `ap front [side] [nose\|nonose]` | show / change the front of the ship: `fore`, `aft`, `port`, `starboard` (relative to the avionics controller) or body axes `+x -x +z -z`; `nose` / `nonose` = fly legs nose-first or in any heading |
| `ap key=value … <command>` | override a numeric controller/navigation setting for one run |
Add `yes` as the last word to skip the "area clear?" question. Routes farther than 100 m from the start are refused unless the last word is
`range=N` (the whole route must then stay in loaded chunks: put a chunk loader on the ship or follow it).

## Diagnostics (hover at ~4 m, CSV in `/ap_data/diag_<name>.csv`)
`ap pushmap` (lateral authority in 8 directions, finds weak sides), `ap gust` (unmodelled disturbances), `ap imbal` (hidden motor imbalances,
integrators must find them), `ap leanstep` (lean angle vs acceleration), `ap yawcal` (yaw authority per rotor), `ap idpulse` (pulse response, no save).

## Tandem and other lean-only ships
A tandem helicopter (two rotors on one axis) cannot make roll torque with thrust: the wizard-free scan detects it and the controller makes roll from the bearing tilts and translates sideways only by leaning. `ap tune` then shows a low lateral authority on that axis and flights turn the ship before long legs so the travel direction uses its strong axis.

### Front of the ship
`ap front` shows the front, how much of the weight the front-side rotors carry, and the compass bearing the front points to; `ap where` prints the bearing too. Nose-first ships turn before every leg of 15 m or more so the front points along the travel direction, unless the measured lateral authority says that direction is clearly weak (then the weak side is kept off the travel axis instead). If the weight distribution of the ship changes (flights print a note when the front-side share moved by more than 10 points), move the front with `ap front <side>`; the saved trim and tune stay valid.

## Safety
* Tilt over 75 % of the limit switches to level mode (position and lean demands dropped, the ship is levelled); only a tilt over 60 deg (or over the limit within 2 m of the ground) cuts the motors; drift, altitude excess, climb/landing time-outs and the time limit land the ship instead.
* A flight lock file is written at take-off; if the computer restarts mid-air (chunk unload, crash) `ap recover` (run from startup) takes over and lands the ship.
* Transit never plans below its floor (default from the ship size), breaches climb and slow down, lean is governed by speed, motor headroom, sag and floor clearance.
* Angular rate comes from the orientation itself; the API's angular velocity (body frame) is only cross-checked.

## Files
In `src/`: `ap.lua` launcher, `ap/main.lua` commands, `ap/lib/` m (math), hw (scan), alloc (allocation), ctl (controller), nav (routes/transit), fly (flight loop),
tune, diag, wizard, airframe, hal (real ship / simulator), sim (rigid-body simulator), selftest. `build.lua` (run in-game) bundles `src/` into `dist/apinstall.lua`: copy the repo to a computer and run `build dist/apinstall.lua <version> src`.
Design notes and measurements: `docs/DESIGN.md`. Measured hardware facts (Sable API shapes, bearing handedness, timing): `docs/field-notes/`. Example flight logs: `data/examples/`.
