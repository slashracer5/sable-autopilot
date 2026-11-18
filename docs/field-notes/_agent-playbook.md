# Agent playbook: working with CC peripherals on a Sable ship

Rules distilled from the 2026-11-16 quadcopter session (about 1300 messages; most of the lost time came from trusting an assumption instead of measuring). Applies to any build that drives hardware through a ComputerCraft bridge MCP (the `cc_*` tools: `cc_exec`, `cc_push`, `cc_batch`, ...).

## 1. Order of operations
1. `cc_computers` -> pick the computer id (several can be connected; every tool needs `computer=` then).
2. `cc_network` (all peripherals incl. the `ship_*` bridge, grouped by type) and read-only getters first. Do not call setters while inventorying.
3. Confirm the ship exists: `sublevel.isInPlotGrid()`, `sublevel.getMass()`.
4. Map the hardware physically before writing a controller (see 3).
5. Dry-run mode (print what you *would* command) before any actuation, then low-power ground tests, then flight.

## 2. Measure, do not infer (every one of these was wrong at first)
| Assumption | Reality |
|---|---|
| `setManualTarget` takes a world vector (docs) | ship-local vector |
| default `right_handed` bearings lift | on bearings placed facing up with the motor below they push down; `left_handed` lifted and `getThrust()` showed lift as negative. The right setting depends on placement orientation, so test it |
| prop positions from compass/yaw maths | wrong; single-prop ground test with the gimbal sensor gave the truth |
| a slow wobble = loose gains | it was the loop running at 0.2 s per step (sequential tick-costing calls: setters and `sublevel.*`) and a 0.2 s plant delay |
| big imbalance from off-centre mass | the real cause was the wrong geometry sign; trim ended up tiny |
| a failed/hung call = broken code | the game was paused/unfocused, or the scan ran during a ship reassembly |

Before trusting any sign or frame claim, build a one-variable test (spin one prop, apply one lean step, one open-loop pulse) and read the result from a sensor.

## 3. Mapping an unknown ship
- List names with `cc_network`; pair motors/transmissions/gearshifts/bearings by powering **one** transmission at level 1 for ~2 s and diffing `getSpeed()` of every gearshift and bearing, then set it back to 0.
- Find each prop's position by spinning it alone and reading the `gimbal_sensor` tilt (or ask the user which side of the ship it is on; a compass guess is not enough).
- Find lift direction on the bench: `getThrust` sign is not lift; ask the user to watch airflow or measure acceleration.
- Record the result in a header comment of the program and in memory.

## 4. Safety for live actuators
- Ask before the first setter, the first spin-up, the first flight. State what you will change and how you will restore it.
- After every test: transmissions 0, motors back to their original RPM (usually 256), manual targets cleared. Verify by reading them back.
- Never call `assemble()` / `disassemble()` on a propeller bearing. Never probe out-of-range values on a live drivetrain (an early range probe on an assembled prop was a mistake the agent had to own up to).
- Keep a hard fault (the project used a tilt cut-out at 40 deg under the user's 45 deg limit) that cuts motors, restores state and lands.
- Hand-off pattern for flights: user keeps Minecraft focused with `Ctrl+T` ready; the program has a time limit and a gentle-landing path so you can intervene between steps.

## 5. Process hygiene
- When analysing a log after a run, check that the run really happened (a compile error once made the analysis re-read the previous flight's log).
- Upload code with `cc_push` (not by pasting the file into `cc_write`); run `selftest` modes on the computer after every edit.
- Long inline shell scripts break: write analysis scripts to files.
- Call out an earlier wrong statement as soon as the data shows it (the agent had to retract "fore is on the north side").

## 6. Where the proven artefacts are
`src/ap/` (the controller and its selftest) and `docs/DESIGN.md` (measurements and the bugs they exposed).
