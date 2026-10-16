# DOVER robot specs

These are the numbers the code depends on. If the hardware changes, update this file first. Then update the constants listed in the **Code** column, then add a line to the change log in `AGENTS.md`.

Status tags:

- **confirmed**: measured or tested on the robot.
- **calc**: derived from confirmed numbers.
- **CAD**: from the STEP model, not the real robot.
- **TODO**: unknown or placeholder.

---

## 1. Controller and pins

| Item | Value | Status | Code (files) |
|---|---|---|---|
| Board | Arduino Nano (ATmega328P, 30 KB usable flash, 2 KB RAM). V2.1: 29,108 B / 1,146 B with avr-gcc here (16 Oct); the IDE reads about 1 KB more flash. | confirmed | n/a |
| Outtake driver | Heatsinked **IN1/IN2 + PWM** board (part number not recorded) | confirmed 14 Oct | n/a |
| Motor dir A | **D8** | confirmed 14 Oct | `OUTTAKE_MA` (V2_1 `RobotConfig.h`, calibration/Slide_Test, Slide_PID_Tune); `MOTOR_A` (calibration/Slide_Encoder_Check) |
| Motor dir B | **D9** | confirmed 14 Oct | `OUTTAKE_MB`, `MOTOR_B` |
| Motor PWM | **D11** | confirmed 14 Oct | `OUTTAKE_PWM`, `MOTOR_PWM` |
| Encoder A / B | **D2 / D3**. These are the Nano's only interrupt pins; never move them. | confirmed | `OUTTAKE_ENC_A/B`, `Encoder enc(2, 3)` |
| Servo board | PCA9685 on I2C (A4 SDA / A5 SCL), address 0x40, 50 Hz (prescale 121) | confirmed | `pcaBegin()` (V2_1, Servo_Offset_Cal) |
| Servo outputs (1-based in code) | 1 = arm right (ch0), 2 = arm left (ch1), 3 = scoop (ch2) | confirmed | `ARM_R_OUTPUT`, `ARM_L_OUTPUT`, `SCOOP_OUTPUT` |
| End LED (R25) | `LED_BUILTIN` (D13). Needs a bright external LED that the camera can see. | TODO | `END_LED_PIN` |

The motor drive logic is: MA high + MB low = one direction; MA low + MB high = the other; both low with PWM 0 = stop. `MotorController` already works this way. Which input is IN1 doesn't matter, because `SLIDE_MOTOR_DIR` (V2_1) / `OUTTAKE_MOTOR_DIRECTION` (calibration sketches) sets the sign.

**The KiCad schematic is out of date.** It shows an IRF3205 DIR+PWM driver, TMC2209 steppers and a different Nano pin map. Use the pins above instead.

## 2. Power

| Item | Value | Status |
|---|---|---|
| Battery | 12–17 V (4S) → 7.5 A fuse → switch | old schematic, verify |
| Servo supply | Buck converter, 5 V → PCA9685 V+ | old schematic, verify |
| Encoder 5 V | Comes from the battery-side rail. **Battery must be ON or the count drifts.** | confirmed (behaviour) |
| Nano supply | The schematic shows buck 5 V going into **VIN**, which sags without USB. It should feed the 5V pin. | TODO: check the board |

## 3. Outtake slide drivetrain

| Item | Value | Status | Code |
|---|---|---|---|
| Motor | goBILDA 5203 Yellow Jacket, 435 rpm, 13.7:1 | confirmed | n/a |
| Encoder resolution | **384.5 counts per gearbox output rev** (x4 quadrature, `Encoder` library). The encoder is on the motor; the gearing to the spool is now 1:1. | confirmed | n/a |
| Gearing to spool | **1:1** (16 Oct; the 1.8:1 step-up was swapped out). Changing it: see AGENTS.md "Workflows". | confirmed 16 Oct | `SLIDE_STEP_UP` = 1.0 (V2_1 `RobotConfig.h`) |
| Counts per spool rev | **384.5** | calc | n/a |
| Spool diameter | **36.15 mm** (113.57 mm per rev) | confirmed | `SLIDE_SPOOL_DIA_MM` |
| Stringing | **1:1** (1 mm of string = 1 mm of slide travel) | confirmed | n/a |
| **mm per count** | π × 36.15 / 384.5 = **0.2954** (3.386 counts per mm). Was 0.5317 with the 1.8:1 step-up. | calc | `SLIDE_MM_PER_COUNT` (V2_1, computed); `MM_PER_TICK` (calibration/Slide_Test, Slide_PID_Tune); `MM_PER_COUNT` (calibration/Slide_Encoder_Check) |
| Encoder direction | **−1**. The raw count goes negative on extend. | confirmed 14 Oct | `SLIDE_ENCODER_DIR` (V2_1); `OUTTAKE_ENCODER_DIRECTION` (Slide_Test, Slide_PID_Tune). Slide_Encoder_Check shows the raw count on purpose. |
| Motor direction | +1 (positive PWM should extend). Check that `j 100 300` extends. | **TODO** | `SLIDE_MOTOR_DIR` (V2_1); `OUTTAKE_MOTOR_DIRECTION` (calibration) |
| Free speed | 2790 counts/s ≈ **824 mm/s** at 12 V; about 1150 mm/s on a full 4S (16.8 V) | calc | n/a |
| Usable speed | About 80% of free speed: ~650 mm/s at 12 V (~2200 counts/s). The 15 Oct runs peaked at ~3360 counts/s ≈ 990 mm/s, so the battery was well above 12 V. `SLIDE_RETRACT_SPEED` (1111 mm/s, was `SLIDE_MAX`) asks for more than that: PWM saturates on RETRACT. | calc | `SLIDE_FAST`, `SLIDE_RETRACT_SPEED` (V2_1 `Sequence.h`, mm/s) |
| Stall pull | 1.83 N·m motor = 1.83 N·m at the spool (1:1) → **about 101 N** of string pull | calc | n/a |
| Moving mass | Not measured (needed for the accel limit and kA) | TODO | n/a |
| Slides | 9 stages × 400 mm, stacked 10 mm apart | CAD | n/a |
| Max safe travel | **End stop at ~1694 mm** (~5735 counts): every 15 Oct full run stalled there during CREEP. Slide_Test has no limit (Tinzen's choice). The stall cut-out is the only end protection. | measured 15 Oct (logs) | `SLIDE_END_STOP_MM` (V2_1 `RobotConfig.h`): not a soft limit, the firmware doesn't use it; the sequence editor warns about targets past it. The stall fallback retracts. |
| Creep start / bucket fully tipped | `X_CREEP` **1666.7 mm**, `X_TIP` **1788.9 mm**: the 15 Oct tuning values (3000 / 3220) re-scaled to the true mm/count, so the motion in counts is unchanged. `X_TIP` is **past the ~1694 mm end stop**, so CREEP ends in a STALL and the FALLBACK skips TIP_HOLD. CAD estimate ~1650 / 1740. | **TODO: set X_TIP below the stop** | `X_CREEP`, `X_TIP` (V2_1 `Sequence.h`) |

V2_1's sequence uses **mm from fully retracted** (zeroed at power-up: slides must be fully in). The controller runs in encoder counts internally. Slide_Test and Slide_PID_Tune take mm on serial.

Gains are tuning values, not specs: `SLIDE_K*` in V2_1 `RobotConfig.h`, in motor encoder counts (they survive a gear-ratio change except `KG` / `KA`). Slide_PID_Tune `g` prints them (named `OUTTAKE_K*`).

## 4. Intake servos

| Item | Value | Status | Code (V2_1 `RobotConfig.h`) |
|---|---|---|---|
| Servos | goBILDA 2000-series, **300° over 500–2500 µs** | believed (Tinzen); `Servo_Offset_Cal` scale check confirms | n/a |
| Arm drive | Two servos on one shaft, mirrored. 40T servo gear → 48T arm gear. | CAD | `ARM_GEAR_RATIO` |
| Arm scale | **8.0 µs per arm degree**; right servo pulse falls as the arm goes out, left rises | calc | `ARM_US_PER_DEG`, `ARM_R_DIR` −1, `ARM_L_DIR` +1 |
| Scoop scale | Direct drive, **6.667 µs per degree** | calc | `SCOOP_US_PER_DEG`, `SCOOP_DIR` +1 |
| Reference pose | **Scoop laid flat on the intake floor** (hard stop). CAD: arm **110.6°**, scoop **−69.4°**. | CAD | `CAL_ARM_DEG`, `CAL_SCOOP_DEG` |
| Pulses at the reference pose | R **910**, L **2105**, scoop **1968** µs, measured 16 Oct with the editor's Calibrate servos tab (was 940 / 2060 / 1900 from the V2 pose). Angles in `Sequence.h` re-worked so the robot moves the same. | confirmed 16 Oct | `ARM_R_CAL_US`, `ARM_L_CAL_US`, `SCOOP_CAL_US` |
| Soft limits | Arm −55…125°, scoop −260…20°, any pulse 500–2500 µs | n/a | `ARM_MIN_DEG` … `PULSE_ABS_MAX` |
| Unloaded speed | ~430 °/s at the servo (estimate): the fastest a servo profile can usefully go (arm 358 °/s through the gears) | estimate | `SERVO_NATIVE_DEG_S`, `ARM_FAST` / `SCOOP_FAST` |

Pulse = reference pulse + direction × scale × (angle − reference angle). Angles: **arm** = degrees forward of vertical (+ = out toward the intake floor); **scoop** = degrees relative to the arm, 0 = CAD pose (blade flat on top, arm vertical). The twin rotates the scoop by +angle about the CAD X axis (direction +1, checked: scoop holds rocks at SCOOP/PICKUP and pours at TRANSFER).

Named poses are sequence values and live in `Sequence.h` (degrees). The tested V2 positions as of 15 Oct, for reference:

| Arm pose | V2 units | Deg | | Scoop pose | V2 µs | Deg |
|---|---|---|---|---|---|---|
| A_START | 940 | −47.4 | | S_START | 2300 | −9.4 |
| A_TRANSFER | 920 | −43.4 | | S_PICKUP | 2100 | −39.4 |
| A_MOVE | 500 | 40.6 | | S_SCOOP | 1900 | −69.4 (flat on the floor) |
| A_PEAK | 350 | 70.6 | | S_CARRY (V2 transfer) | 1800 | −84.4 |
| A_DOWN | 200 | 100.6 | | S_DOWN | 700 | −249.4 |
| A_SCOOP | 150 | 110.6 (flat on the floor) | | S_POUR (new, untested) | 1667 | −104.4 |

## 5. Geometry (CAD frame: mm, Z up)

| Item | Value | Status |
|---|---|---|
| Envelope | **393 × 370 × 401.3 mm** (height is to the bucket top) | CAD |
| Start cube (R14) | 400 mm, +0/−1. The CAD height is 1.3 mm over. **Measure the real robot.** | TODO |
| Arm pivot | (0, 0, 120); arm length pivot to scoop pivot 234 mm | CAD |
| Scoop pivot | (0, 0, 354) | CAD |
| Slide axis | 32° from horizontal, 6.6° yaw toward the hopper | CAD |
| Real slide angle | **Lower than CAD** (Tinzen). The twin uses about 17° (tilt +14.9° about (3.2, −231.9, 68.1)), estimated from where the bucket reaches the hopper rim. | TODO: measure |
| Bucket | Tips passively when it touches the top pole | confirmed |

## 6. Field (as used by the twin; check against the rulebook)

| Item | Value |
|---|---|
| Hopper centre | Field (2102.5, 900) mm |
| Hopper rim height | 732 mm |

## 7. Open items (measure → update here → update code → log)

1. Servo calibration (`calibration/Servo_Offset_Cal`), including the scale check.
2. Motor direction (does `j 100 300` extend?).
3. `X_CREEP` / `X_TIP`: jog until the bucket touches the pole / has tipped.
4. Slide gains: `calibration/Slide_PID_Tune`, then `SLIDE_K*`.
5. `S_CARRY`, `S_POUR`, `PICKUP` time: tune in the sequence editor (Step tab, or drag the ◆ on the timeline with Drives: Robot · live).
6. Real slide angle, and max safe travel.
7. Real robot height against the 400 mm cube.
8. Motor driver part number; Nano VIN vs 5V feed.
9. Moving mass of the slides.
