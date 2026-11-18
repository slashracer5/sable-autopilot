# Example flight logs

A small, representative selection. The full set of 44 logs from the development flights is attached to the GitHub Releases as `flight-logs.zip`.

| file | ship | what |
|---|---|---|
| `quad_fly1.csv` | quadcopter (4 rotors) | first hover with the generic code (`ap fly 2 8`) |
| `tri_fly2.csv` | tricopter (3 rotors) | first stable hover after the body-frame rate fix |
| `heli_fly1.csv` | tandem helicopter | hover |
| `heli_tune1.csv` | tandem helicopter | `ap tune` identification flight |
| `tri_diag_pushmap.csv` | tricopter | `ap pushmap`: lateral authority in 8 directions |
| `quad_transit943_probe.csv` | quadcopter | ~1 km transit with the final-approach fix |
| `heli_transit14900_drag.csv` | tandem helicopter | 14.9 km transit, peak 142.9 m/s |
| `heli_airframe.json` | tandem helicopter | an `airframe.json` as the wizard + tune write it |

Positions `x`, `z` (and `refx`, `refz`) are metres relative to the first sample of each log (the take-off point); altitude `y` is the absolute world height, because air pressure depends on it. In `heli_airframe.json` the calibration `stamp` is zeroed.

CSV columns (first row is the header): `t` seconds, `phase`, `y` altitude, `vy`, `tilt` degrees, controller outputs and the per-rotor RPMs `rpm1..rpm6`. Column sets differ a little between flight kinds (transit logs carry route and lag columns); read the header row.
