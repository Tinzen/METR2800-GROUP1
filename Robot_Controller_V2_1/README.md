# Robot Controller V2.1 (robot v2, servo intake)

Same state machine as `Robot_Controller_V2`, made non-blocking and fully wired:

```
DOWN -> SCOOP -> PICKUP -> TRANSFER -> BEFORE_EXTEND -> OUTTAKE_EXTEND -> OUTTAKE_RETRACT -> DONE
```

Sketch folder files (all required):
`Robot_Controller_V2_1.ino`, `MotorController.*`, `MotionProfile.*`, `PIDController.*`, `ServoAxis.*`
Libraries: **Encoder** (Paul Stoffregen) and **Adafruit PWM Servo Driver** (installs Adafruit BusIO).
Board: Arduino Nano (ATmega328P). Uses about 23.4 KB flash and 1.05 KB RAM.

## What changed vs V2

| Area | V2 | V2.1 |
|---|---|---|
| Compiles | No (outtake cases half-ported) | Yes |
| Arm + scoop | Instant servo steps, `delay()` | `ServoAxis` on the v1 `MotionProfile` (accel/cruise/decel) |
| SCOOP sweep | 2 s blocking loop | Same path, one profiled move, non-blocking |
| Outtake | Never ran (BEFORE_EXTEND went to DONE) | Profile + PID + feedforward, closed loop on encoder D2/D3 |
| Transfer | No vibration | v1 scoop vibration back (±50 µs, 50 ms toggle, ≥1 s) |
| Visibility | One print every 100 ms | 50 Hz CSV telemetry + events, run summary, config dump |
| Safety | None | Stall detect, wiring-direction check, no-motion check, I2C timeout, servo-board check, watchdog, `x` abort |
| R25 end signal | Missing | LED on `END_LED_PIN` at DONE |

State timings are V2's, kept as **minimum** times. Each state also waits for its motion profile to finish. Lower the `*_MIN_MS` values to go faster.

`MotionProfile`, `MotorController` and `PIDController` only gained additive features (profile velocity getter, optional feedforward, open-loop mode, velocity estimate). With the new gains at 0 they behave like v1.

## Test workflow (USB tether)

The Nano restarts every time you upload, plug in USB, or open the serial port (Serial Monitor or the capture script). With the default `WAIT_FOR_GO = false` the run starts on every restart, exactly like the power switch at competition.

**Bench mode (recommended while testing):** set `WAIT_FOR_GO = true` and upload.
1. Battery ON.
2. `python tools/capture_log.py` (needs `pip install pyserial`). The Nano resets, holds the start pose and blinks the end LED.
3. Type `g` + Enter to start. `x` + Enter aborts.
4. It stops 1 s after DONE and writes `logs/run_<time>.csv` plus `logs/run_<time>.html`. Double-click the HTML to see the run.

Set `WAIT_FOR_GO = false` again before competition. Every log header shows `wait_for_go=` so you can check.

Close the Arduino Serial Monitor first. Only one program can own the COM port.

For the slides on their own, use `Slide_Test` (see its README).

## First-time calibration (do in this order)

Set `RUN_MODE = RUN_CONSOLE`, upload, run `python tools/capture_log.py --keep-open`, then:

1. **Directions.** Type `j 100 300`. The slide must extend and `oP` must go up.
   - Ticks go down: flip `OUTTAKE_ENCODER_DIRECTION`.
   - Slide retracts: flip `OUTTAKE_MOTOR_DIRECTION`.
   - The firmware cuts the motor with `DIR_MISMATCH` or `NO_MOTION` if this is wrong during a profiled move.
2. **Extend target.** Retract fully and type `z` (zero). Jog out with `j 120 300` until the bucket has fully tipped on the hopper rim, then read `oP`. Put that number in `OUTTAKE_EXTEND_TICKS`. The default of 1000 is deliberately short.
3. **Arm trim.** Use `a 940`, `a 500` etc. If either arm servo hums or gets warm at rest, adjust `ARM_R_TRIM_US` / `ARM_L_TRIM_US`.
4. **Feedforward.** Run `o 3000 800`, `o 0 800`, `o 3000 1600`, `o 0 1600` (use a target inside your travel). Open the log and copy kS / kV / kG from the viewer's Feedforward fit panel. Then raise `OUTTAKE_KP` until tracking error is small without oscillation.
5. Set `RUN_MODE = RUN_FULL` for competition.

`RUN_OUTTAKE_ONLY` and `RUN_INTAKE_ONLY` run half the sequence for focused testing.

## Servo angles in degrees

goBILDA 300° servos give exact rates:
- Scoop: 6.667 µs per degree.
- Arm: 40T → 48T gears give 8.0 µs, which is exactly **5 position units, per arm degree**.

| Constant | Default | Meaning |
|---|---|---|
| `ARM_VERTICAL_POS` | 650 | arm position with the posts vertical |
| `SCOOP_ZERO_US` | 2300 | scoop pulse in the start/CAD pose |

Both zeros were solved from the CAD. With them, the scoop just touches the floor at A_DOWN, and the start pose fits the cube. The current positions come out as round numbers:

| Arm | Angle (forward of vertical) |
|---|---|
| A_START | −58° |
| A_TRANSFER | −54° |
| A_MOVE | 30° |
| A_PEAK | 60° |
| A_DOWN | 90° |
| A_SCOOP | 100° |

| Scoop | Angle (from start pose) |
|---|---|
| S_PICKUP | −30° |
| S_SCOOP | −60° |
| S_TRANSFER | −75° |
| S_PEAK | −180° |
| S_DOWN | −240° |

Calibrate in RUN_CONSOLE:
- `A 0` should stand the posts vertical; check with a phone level. If not, nudge `ARM_VERTICAL_POS`.
- `S 0` is the start pose.

Positions can be written as `ARM_POS_FROM_DEG(90)` / `SCOOP_US_FROM_DEG(-75)`. The existing values are unchanged. The twin reads these constants from each log, so calibrating the firmware calibrates the twin.

## Where the time goes (default config, simulated)

| State | s | What sets it |
|---|---|---|
| DOWN | 1.42 | arm waits for the scoop profile (0.73 s), then the arm profile (0.68 s). V2 started the arm at a fixed 500 ms |
| SCOOP | 2.00 | `SCOOP_SWEEP_MS` |
| PICKUP | 1.00 | `PICKUP_MIN_MS` (the scoop move itself takes 0.23 s) |
| TRANSFER | 2.00 | `TRANSFER_MIN_MS`, vibration ≥ `VIBRATE_MIN_MS` |
| BEFORE_EXTEND | 1.00 | `BEFORE_EXTEND_MIN_MS` (the arm move takes 0.5 s) |
| OUTTAKE_EXTEND | ~4.1 | profile (~2.6 s) + `OUTTAKE_DUMP_MS` (1.5 s) |
| OUTTAKE_RETRACT | ~2.6 | profile |

Total about 14.1 s (boot to DONE). These figures come from the software-in-the-loop sim (`tools/sil_sim`), not a real run. Replace them with your own logs.

### Speed-ups, biggest first (each is a constant at the top of the sketch)

1. `OUTTAKE_DUMP_MS` 1500. Film a dump at 240 fps and set it to the time the last rock takes to clear the bucket, plus about 150 ms.
2. `TRANSFER_MIN_MS`, `PICKUP_MIN_MS`, `BEFORE_EXTEND_MIN_MS`. These are V2's fixed waits. The moves themselves finish well before them (0.9 s, 0.23 s, 0.5 s). Set them to 0 and keep `VIBRATE_MIN_MS` as the real transfer time.
3. The slide's `OUTTAKE_*_MAX_VEL`. Raise it until the log shows PWM saturating at 250 during cruise.
4. `SCOOP_MAX_VEL` if the scoop servo is a speed variant. DOWN waits for it.
5. Check boot delay. If you have to select "ATmega328P (Old Bootloader)" to upload, the board's old bootloader may wait up to ~1 s after power-on before `setup()` runs. Optiboot skips that wait on power-up. Measure switch-flip to first servo move with phone slow-mo before deciding to re-flash (needs an ISP, e.g. another Arduino).

With the first two applied, the sim's "faster" example run takes 12.0 s instead of 14.1 s.

## Telemetry format

```
D,t,st,ph,aT,aC,sT,sC,sO,oT,oS,oP,oV,oPWM,oFF,fl,lm   every 20 ms
S,t,STATE,index                                        state change
E,t,CODE,value                                         event, see below
R,t,run_ms                                             run finished
#...                                                   header / config (config printed after DONE)
```

Column meanings are documented above `logTelemetry()` in the sketch.

| Event | Meaning |
|---|---|
| `ARRIVED` | Slide reached its target (±25 ticks) |
| `SETTLED_SHORT` | Slide stopped within 150 ticks of target and the run moved on. Tune kG / kS. |
| `STALL` | Pushing ≥90 PWM but not moving (bucket jammed or end stop). Run treats it as arrived. |
| `TIMEOUT` | Move took longer than its profile + 1.5 s |
| `NO_MOTION` | Stalled without the encoder moving. Motor reversed, unpowered, or encoder unplugged. Run aborts. |
| `DIR_MISMATCH` | Pushing one way, running away the other. Motor or encoder direction flipped. Run aborts. |
| `STALL_CUT` | Console move stalled, motor cut |
| `I2C_TIMEOUT` | Servo-board bus glitch, bus was reset |
| `NO_SERVO_DRIVER` | PCA9685 not found at boot, run refused |
| `WATCHDOG` | Run exceeded 60 s and was stopped |
| `VIB_ON` / `VIB_OFF` | Transfer vibration start/stop (value = amplitude / duration) |

**Power note:** if the Nano browns out and resets when the motor starts, it restarts the run and re-zeros the encoder wherever the slide is. Give the Nano its own regulator, or decouple it from the motor supply.
