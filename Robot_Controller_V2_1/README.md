# Robot_Controller_V2_1

The competition program for the V2 robot (servo intake, goBILDA slide motor). It runs a **table of steps** from `Sequence.h`: V2's tested intake, V1's transfer (carry, pour, vibrate) and a profiled outtake that blends from fast into a slow creep so the bucket tips gently.

| File | What's in it | Who edits it |
|---|---|---|
| `Sequence.h` | **The state machine.** Named angles (degrees), slide positions (mm), speeds, and one row per step. | You, or `tools/sequence_editor.html` (saves straight to it) |
| `RobotConfig.h` | Pins, slide gear ratio and spool, servo calibration, slide gains, safety limits. Must match `ROBOT_SPECS.md`. | Calibration results, spec changes |
| `Robot_Controller_V2_1.ino` | The step runner, safety monitors, telemetry, serial console. No tuning numbers. | Code changes only |
| `SequenceTypes.h` | The keywords used in `Sequence.h` (`HOLD`, `WAIT_MS` ...). | Nobody |
| `LOGGING.md` | First-time guide: Python + pyserial setup, logging a run with `tools/capture_log.py`, troubleshooting. | |
| `MotionProfile`, `MotorController`, `ServoAxis`, `PIDController` | Trapezoid profiles (with blending), slide PID + feedforward, servo axes in degrees. | Code changes only |

## The sequence

| | Step | What happens | Ends when |
|---|---|---|---|
| Intake (V2) | `SCOOP_DOWN` | scoop to `S_DOWN` at `SCOOP_FAST` | 300 ms |
| | `ARM_DOWN` | arm to `A_DOWN` at `ARM_FAST` | 500 ms |
| | `SWEEP_UP` | arm lifts to `A_PEAK` (62.1 °/s) while the scoop turns `S_DOWN` → `S_SCOOP` (156.1 °/s) | arm arrived |
| | `SWEEP_DOWN` | arm drops to `A_SCOOP` (75 °/s); the scoop's move carries on | all arrived (arm **and** the scoop's move) |
| | `WAIT` | nothing moves | 120 ms |
| | `PICKUP` | scoop to `S_PICKUP` (lip up) | 260 ms (V1 used 100) |
| Transfer (V1) | `LIFT` | arm lifts to `A_TRANSFER` (222.1 °/s), scoop still at pickup | 100 ms in |
| | `CARRY` | scoop to the carry angle `S_CARRY` (210.6 °/s) while the arm keeps lifting | arm arrived |
| | `POUR` | scoop tips to `S_POUR` over the bucket | all arrived |
| | `VIBRATE` | scoop shakes ±`VIBRATE_DEG` at 10 Hz | 590 ms |
| Outtake | `EXTEND` | arm to `A_MOVE`, scoop to `S_DOWN`, slides at `SLIDE_FAST` to `X_CREEP` | all arrived |
| | `CREEP` | slides carry on at `SLIDE_CREEP` (no stop in between) so the bucket tips slowly | all arrived (`X_TIP` is past the end stop now: see the editor's warning) |
| | `TIP_HOLD` | hold while the rocks fall | 1500 ms |
| | `RETRACT` | slides back to 200 mm at `SLIDE_RETRACT_SPEED` / `_ACCEL` / `_DECEL` | all arrived |

<table><tr>
<td><img src="img/seq_SWEEP.png" width="230" alt="Twin: end of the sweep, scoop flat on the floor"><br>SWEEP_DOWN (end of the sweep)</td>
<td><img src="img/seq_PICKUP.png" width="230" alt="Twin: pickup, lip up"><br>PICKUP</td>
<td><img src="img/seq_CARRY.png" width="230" alt="Twin: lift with the scoop at carry"><br>LIFT / CARRY</td>
<td><img src="img/seq_POUR.png" width="230" alt="Twin: pour over the bucket"><br>POUR</td>
</tr><tr>
<td><img src="img/seq_ARM_DOWN.png" width="230" alt="Twin: arm down, scoop folded back"><br>ARM_DOWN</td>
<td><img src="img/seq_VIBRATE.png" width="230" alt="Twin: vibrate"><br>VIBRATE</td>
<td><img src="img/seq_CREEP.png" width="230" alt="Twin: slides creeping at the hopper"><br>CREEP (CAD slide numbers)</td>
<td><img src="img/seq_RETRACT.png" width="230" alt="Twin: retract"><br>RETRACT</td>
</tr></table>

Pictures come from the twin with CAD estimates; the twin's angles are only exact after the servo calibration.

**Rules for a row** (also in the comments of `Sequence.h`):
- ARM / SCOOP: an angle name, a number, or `HOLD`. `HOLD` means "don't touch it": a move started in an earlier step keeps going. That is how steps overlap (the arm keeps lifting through `CARRY`, the arm clears during `EXTEND`).
- Every move is a motion profile. Each ARM / SCOOP has a speed (deg/s, 0 = `ARM_SPEED` / `SCOOP_SPEED`; `ARM_FAST` / `SCOOP_FAST` = the servo's own top speed, the old INSTANT), and each axis speeds up at its `_ACCEL` and slows down at its `_DECEL` (`ARM_`, `SCOOP_`, `SLIDE_`, `SLIDE_RETRACT_`). A new target takes over a running move smoothly; HOLD lets it carry on. SLIDE picks a named speed (`SLIDE_FAST`, `SLIDE_CREEP`, `SLIDE_RETRACT_SPEED`); it uses `SLIDE_ACCEL` / `SLIDE_DECEL`, or `SLIDE_RETRACT_ACCEL` / `SLIDE_RETRACT_DECEL` with the retract speed. (The MOVE and ARM VIA columns were removed 16 Oct; SWEEP became SWEEP_UP + SWEEP_DOWN.)
- SLIDE: mm from fully in, or `HOLD`. Two slide rows in a row that keep going the same way **blend**: the first slows to the second's speed instead of stopping.
- NEXT WHEN: `WAIT_MS(ms)`, `WAIT_ARRIVED(ms)` (**every** axis has stopped: arm, scoop, and the slides once they've been used this run, including moves started in earlier steps), `WAIT_ARM_ARRIVED(ms)` / `WAIT_SCOOP_ARRIVED(ms)` / `WAIT_SLIDE_ARRIVED(ms)` (just that axis), `WAIT_ARM(%)`, `WAIT_SCOOP(%)`, `WAIT_SLIDE(%)`. For the arrived ones, `+ms` waits that long after arriving and `-ms` moves on that long *before* arriving (from the profile's planned finish time), so the next move blends in. (Before 16 Oct, `WAIT_ARRIVED` only waited for the axes the step itself moved.)
- A.ACC% A.DEC% S.ACC% S.DEC% (last four, optional): this row's arm / scoop move speeds up / slows down at that % of `ARM_` / `SCOOP_` `ACCEL` / `DECEL` (1–255, 100 or left out = those). Only that row's move; HOLD axes ignore it.
- FLAG (last column, optional, for testing): `STEP_RUN` (normal), `STEP_SKIP` (the row doesn't run), `STEP_STOP` (the run ends after the row: slides to 0 mm with the arm / scoop at `A_RESET` / `S_RESET`, then `A_START` / `S_START` once the slides are in, at jog speed; logs `E,STOP_HERE,<row>`, then DONE). Leave every row on `STEP_RUN` for the competition; the editor shows a red TEST chip while one is set.
- OR AFTER (optional, before the % columns): a time limit in ms. The step moves on after it even if NEXT WHEN never comes true, and logs `E,STEP_LIMIT,<step>`. `0` (or left out) = no limit. It scales with `k` like the other times.

**Programs:** `programs/*.h` hold versions of the whole sequence (Max speed, Slow 20 s, Slow 30 s); `Sequence.h` is a copy of the one picked in the editor's Program menu (`#define PROGRAM_NAME`), and it's the only one compiled. The robot prints `#PROGRAM,<name>` at start-up.

## Editing and tuning

1. Open `tools/sequence_editor.html` in Chrome or Edge → **Open robot folder** → pick this folder. Edit, watch the twin and the predicted run time, **Save** (Ctrl+S).
2. Upload `Robot_Controller_V2_1.ino`.
3. First runs on the robot: set `SPEED_PCT_AT_BOOT = 25` (slow motion) and/or `STEP_MODE_AT_BOOT = true` (pauses after every step, `n` = next), or use the console commands below.
4. Log runs with `python tools/capture_log.py` and open the `.html` it writes. **Never done it?** [`LOGGING.md`](LOGGING.md) walks through installing Python and pyserial, then logging a run.

**Tune it in the editor first**, then confirm on the robot: see [`SEQUENCE_TUNING.md`](../SEQUENCE_TUNING.md). In the Step tab, the ↑ ↓ keys in a step's value box nudge a pose live (the twin, and the robot when connected, follow). On the timeline, drag a ◆ up/down for the pose and sideways for a servo move's speed. The pose sliders are at the top of Angles & values.

**Live link** (check poses by eye on the real robot): upload as it is, close the serial monitor, battery on, editor → **Connect robot** (top right; ▾ next to it = "hold still and wait"). The same sliders now move the robot. On the timeline, Drives **Robot · live** makes the robot copy the twin while you scrub or play, so edits can be tried without uploading (`P` command below); **Robot · uploaded** runs this program's own copy (`r <step>`). Every run is saved to `logs/`. Save and re-upload for the real run.

**Bench handshake:** at every power-up the robot waits `BENCH_WINDOW_MS` (150 ms) for a computer to send `b`. The editor sends it when you connect with **Hold still**, so you never need to change `RUN_MODE`. With nothing attached (competition) it runs as normal. The 150 ms is added to every run.

## Test settings (top of the .ino)

| Setting | Bench | Competition |
|---|---|---|
| `RUN_MODE` | leave `RUN_FULL`: the editor holds the robot still on connect. `RUN_CONSOLE` = always wait for commands | **`RUN_FULL`** |
| `WAIT_FOR_GO` | `true` = wait for `g` after the USB reset | **`false`** |
| `FIRST_STEP` / `LAST_STEP` | run part of the sequence (e.g. 0..7 = intake + transfer only) | **0 / 255** |
| `STEP_MODE_AT_BOOT` | `true` = pause after each step | **`false`** |
| `SPEED_PCT_AT_BOOT` | 25 or 50 for first tries | **100** |

## Serial console (115200)

`x` stop · `g` run · `r <n>` run from step n · `t` step mode on/off · `n` next step · `k <pct>` speed % · `l` list steps · `p` config ·
`A <deg>` arm · `S <deg>` scoop · `Q` pose · `o <mm> [mm/s]` slide move · `j <pwm> <ms>` slide jog · `z` zero slide · `c` coast · `v` vibrate ·
`b` bench: stop and wait for commands · servo calibration: `U <1-3> <µs>` raw pulse, `L <1-3>` limp (`L 0` = all), `M` back to degrees (holds the reference pose)
`P <arm> <scoop> [mm] [mm/s] [vib]` follow the editor's timeline (sent ~25 times a second by the editor's Drives: **Robot · live**): servos straight to the angles at their own speed, slides re-planned toward mm arriving at mm/s (mm < 0 = leave the slides), vib > 0 = shake ±vib°. No reply line. `M <arm> <scoop>` = back to degrees holding that pose.

Jog commands work in bench mode (`b`, or `RUN_CONSOLE`), while `WAIT_FOR_GO` is waiting, and after a run has finished. During a run only `x`, `t`, `n` and `b` are accepted.

## Safety and events

- **Slide stall** (pushing ≥ 90 PWM, slower than 50 counts/s for 200 ms) or **timeout** (profile time + 1.5 s): runs the `RETRACT` step once, then stops. Events `STALL`, `TIMEOUT`, `FALLBACK`. Watched until the slide move arrives, even after the row that started it has ended. Outside a run (console jogs, the editor's hand jog) a stall just cuts the motor (`STALL_CUT`).
- **Wiring check**: driving one way but running away fast the other → motor off, `DIR_MISMATCH`.
- `SETTLED_SHORT`: the slide stopped close to (but not at) its target and the step moved on. Tune `SLIDE_KG` / `SLIDE_KS`.
- `STEP_LIMIT,<step>`: that step's OR AFTER time ran out before its NEXT WHEN came true, so it moved on.
- `STOP_HERE,<step>`: a `STEP_STOP` row ended the run (test flag); the robot goes back to the start pose, then DONE.
- 60 s watchdog, I2C timeout recovery, `NO_SERVO_DRIVER` (won't run the slides with no servo board).
- DONE: motor off, servos hold, LED on.

## Changing the slide gear ratio

Edit `SLIDE_STEP_UP` in `RobotConfig.h` (and the spec in `ROBOT_SPECS.md`). Slide positions and speeds are in mm, so `Sequence.h` doesn't change, **unless** those values were found on the robot while the ratio in the code was wrong. Then they are really counts in disguise: scale them by old ÷ new mm/count so the motion stays the same (done on 16 Oct for 1.8:1 → 1:1). Gains are in motor encoder counts, so `KS`/`KV` stay valid; re-check `KG`/`KA` with `calibration/Slide_PID_Tune`. The editor shows the new free speed; keep `SLIDE_FAST`/`SLIDE_RETRACT_SPEED` under about 80% of it.

## Telemetry

Same `D` columns as before so `tools/run_viewer.html` still works: arm logged as 0–1000 (650 = vertical, 5 per degree), scoop as µs-equivalent (2300 = 0°, 6.667 per degree), slides in encoder counts. `#STATES` carries the step names, `S` lines mark each step, `#CFG` lists calibration and gains.

Flash 29,108 B of 30,720 B, RAM 1,146 B of 2 KB with avr-gcc 7.3 and a stand-in Encoder library (16 Oct); the IDE showed about 1 KB more flash than this build, so check its number after uploading (no Adafruit servo library: a 30-line PCA9685 driver with the same 50 Hz setup).
