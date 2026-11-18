# Field notes (hand-written, measured in game)

Facts about ComputerCraft + Create Aeronautics / Sable / Create Avionics that were **measured in a running game**, not copied from documentation. Where a note contradicts the mods' own docs, the note is what happened on the test ships (versions below).

| File | Topic |
|---|---|
| `ship-bridge-and-naming.md` | how `ship_*` peripherals appear, naming, instability of names |
| `propeller-bearing-and-drivetrain.md` | bearing / transmission / motor / gimbal behaviour, handedness sign, `setManualTarget` frame |
| `sable-sublevel-api.md` | return shapes of `sublevel` / `aero` / `quaternion` / `matrix` |
| `cc-runtime-and-timing.md` | yielding, parallel reads, delay, Lua limits, JSON |
| `_agent-playbook.md` | procedure and checklists for AI agents driving hardware |

Measured on Create: Avionics 0.7.0, CC: Tweaked 1.120.2, CC: Sable 1.3.4, Create Aeronautics 1.3.2 (Minecraft 1.21.1, NeoForge). Re-check anything that depends on a mod after it updates.

## Adding a fact
- Put it in the matching file with the version it was measured on and how it was established (test, sensor, observation). Mark anything unverified as such.
- If a fact corrects the docs, say so explicitly ("the docs say X, measured Y").
