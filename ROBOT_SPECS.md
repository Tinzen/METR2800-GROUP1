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
| Board | Arduino Nano (ATmega328P, 30 KB usable flash, 2 KB RAM). V2.1 uses 28.0 KB / 1.16 KB. | confirmed | n/a |
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
| Encoder resolution | **384.5 counts per gearbox output rev** (x4 quadrature, `Encoder` library). The encoder sits before the step-up. | confirmed | n/a |
| Step-up to spool | **1.8:1** (the spool turns faster than the motor). Changing it: see AGENTS.md "Workflows". | confirmed | `SLIDE_STEP_UP` (V2_1 `RobotConfig.h`) |
| Counts per spool rev | 384.5 / 1.8 = **213.6** | calc | n/a |
| Spool diameter | **36.15 mm** (113.57 mm per rev) | confirmed | `SLIDE_SPOOL_DIA_MM` |
| Stringing | **1:1** (1 mm of string = 1 mm of slide travel) | confirmed | n/a |
| **mm per count** | π × 36.15 / 213.6 = **0.5317** (1.881 counts per mm) | calc | `SLIDE_MM_PER_COUNT` (V2_1, computed); `MM_PER_TICK` (calibration/Slide_Test, Slide_PID_Tune); `MM_PER_COUNT` (calibration/Slide_Encoder_Check) |
| Encoder direction | **−1**. The raw count goes negative on extend. | confirmed 14 Oct | `SLIDE_ENCODER_DIR` (V2_1); `OUTTAKE_ENCODER_DIRECTION` (Slide_Test, Slide_PID_Tune). Slide_Encoder_Check shows the raw count on purpose. |
| Motor direction | +1 (positive PWM should extend). Check that `j 100 300` extends. | **TODO** | `SLIDE_MOTOR_DIR` (V2_1); `OUTTAKE_MOTOR_DIRECTION` (calibration) |
| Free speed | 2790 counts/s ≈ **1480 mm/s** at 12 V | calc | n/a |
| Usable speed | About 1100–1200 mm/s under load. Keep profile max velocity at or below ~2200 counts/s (~1170 mm/s). | calc | `SLIDE_FAST`, `SLIDE_MAX` (V2_1 `Sequence.h`, mm/s) |
| Stall pull | 1.83 N·m motor → 1.02 N·m at the spool → **about 56 N** of string pull | calc | n/a |
| Moving mass | Not measured (needed for the accel limit and kA) | TODO | n/a |
| Slides | 9 stages × 400 mm, stacked 10 mm apart | CAD | n/a |
| Max safe travel | Unknown. Slide_Test has no limit (Tinzen's choice). The stall cut-out is the only end protection. | TODO | V2_1 has no soft limit either; the stall fallback retracts. |
| Creep start / bucket fully tipped | `X_CREEP` 450 mm, `X_TIP` 530 mm are **safe placeholders**. The twin estimates the hopper rim at about 1740 mm (depends on the real slide angle). Measure with calibration/Slide_Test (README step 2). | **TODO** | `X_CREEP`, `X_TIP` (V2_1 `Sequence.h`) |

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
| Pulses at the reference pose | R **940**, L **2060**, scoop **1900** µs: the V2 SCOOP pose, which lies flat on the floor (Tinzen, 15 Oct). Refine with `Servo_Offset_Cal`. | stated; **refine with cal** | `ARM_R_CAL_US`, `ARM_L_CAL_US`, `SCOOP_CAL_US` |
| Soft limits | Arm −55…125°, scoop −260…20°, any pulse 500–2500 µs | n/a | `ARM_MIN_DEG` … `PULSE_ABS_MAX` |
| Unloaded speed | ~430 °/s at the servo (estimate, for `INSTANT` moves) | estimate | `SERVO_NATIVE_DEG_S` |

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
5. `S_CARRY`, `S_POUR`, `PICKUP` time: tune in the sequence editor (Live link).
6. Real slide angle, and max safe travel.
7. Real robot height against the 400 mm cube.
8. Motor driver part number; Nano VIN vs 5V feed.
9. Moving mass of the slides.
