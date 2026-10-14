# Notes for agents / teammates

Read this before changing anything in the outtake or slide code.

## Outtake slide drivetrain

- Motor: goBILDA 5203 Yellow Jacket, **435 rpm** (13.7:1 gearbox).
- Encoder: **384.5 counts per gearbox output rev** (quadrature, as read by the `Encoder` library). It is on the motor, before the step-up.
- **1.8:1 step-up** from the motor output to the slide spool, so the spool turns faster than the motor.
  - Spool speed: 435 x 1.8 = about **783 rpm** with no load.
  - Counts per spool rev: 384.5 / 1.8 = about **213.6**.
  - Max encoder rate with no load: 435 / 60 x 384.5 = about **2790 counts/s**. A profile `maxVel` above about 80% of this (about 2200) will saturate the PWM under load.
- Spool diameter: **36.15 mm**, so 113.57 mm of cable per spool rev.
- **mm per encoder count = pi x 36.15 / 213.6 = 0.5317 mm** (1 mm = 1.881 counts). **Stringing is 1:1** (confirmed by Tinzen): 1 mm of string reeled in = 1 mm of slide travel, so this conversion applies directly to the slide.
- Max slide speed with no load: 435 x 1.8 = 783 rpm at the spool, x 113.57 mm/rev = **about 1480 mm/s** (at 12 V; more on a full 4S battery at PWM 255). Under load, the usable speed is about 1100-1200 mm/s.
- Motor stall torque is about 18.7 kg·cm (1.83 N·m) at 12 V. Through the 1.8:1 step-up that's 1.02 N·m at the spool, or **about 56 N of string pull at stall**. Max acceleration = 56 N / moving mass (unmeasured), minus friction.
- **Encoder direction:** the raw count goes **negative when the slides extend**, so `OUTTAKE_ENCODER_DIRECTION = -1` (set in `Slide_Test`, tested 14 Oct 2026). `Robot_Controller_V2_1` still has `+1` and needs the same flip.
- Motor direction (`OUTTAKE_MOTOR_DIRECTION = 1` in `Slide_Test`) has not been confirmed yet. Check that `j 100 300` extends the slide (with pins 8/9/11).

## Units

- Internally all outtake positions are **encoder counts at the motor**, counted from wherever `z` (zero) was last pressed.
- `Slide_Test` serial commands take **mm** (`m`, `mr`, `ext`, `vel`, `acc`) and convert with `MM_PER_TICK`. The `p` paste output stays in counts, because `Robot_Controller_V2_1` uses counts.
- `Slide_Test` has **no travel limit** (Tinzen removed it on purpose, 14 Oct 2026). Only targets below 0 are clamped. The stall cut-out is the only end-of-travel protection. The `ext` default of 250 mm is a placeholder.

## Test sketches

- `Slide_Encoder_Check/`: about 80 lines. Prints mm and counts. `+` / `-` pulse the motor. Uses the raw count, so extending reads negative. `MM_PER_COUNT = 0.5317`.
- `Slide_Test/`: smooth moves in mm (`m 150`), gain tuning over serial, and 50 Hz logging for `tools/capture_log.py`. See its README for the workflow.
- `Slide_PID_Tune/`: separate tuning sketch. `b <a> <b> [n]` bounce test (profiled) and `step <mm>` (pure PID) print per-move and average scores (peak lag, overshoot, settle, final error). Live gains; `g` prints paste-ready V2_1 lines. Same pins, units and D-line format as `Slide_Test`.

**Confirmed working pins (14 Oct 2026):** outtake motor **MA=8, MB=9, PWM=11**; encoder **A=2, B=3** (interrupt pins). These match the original `Robot_Controller` (V1) outtake pins, not the schematic. See "Electrical" below.

## Known gotchas

- **Battery off makes the encoder count drift by itself** (seen 14 Oct 2026). With USB only, the count ran away while the slide was still, and `Slide_Test` moves aborted. It worked once the battery was on. Likely cause: the encoder's 5 V or ground comes from the battery-side rail. Always test with the battery ON. If the count drifts, check the battery first.

## Electrical

> **The KiCad schematic (shared 14 Oct 2026) is OUT OF DATE.** Tinzen confirmed it shows an old motor driver, and its Nano pin map doesn't match the robot. The real hardware is a custom green mainboard carrying the Nano, a PCA9685 servo board, a buck converter (6R8 inductor, blue LED, screw-terminal output) and a heatsinked motor driver board. Trust the "Confirmed working" pins above over anything below.

**Actual outtake motor driver (photo, 14 Oct 2026):** heatsinked driver board with **two direction inputs + PWM** (IN1/IN2 + PWM style, like L298N / VNH-type drivers). The exact part number hasn't been recorded yet. **MA (D8) and MB (D9) are BOTH needed**: MA high/MB low = one direction, MA low/MB high = the other, and both low with PWM 0 = stop (brake or coast depending on the chip). This matches `MotorController` as written, so no refactor is needed.

### Old schematic (for reference only, shared 14 Oct 2026)

**Power**
- Battery is 12-17 V, then a 7.5 A fuse (F1), then a DPDT switch (SW1A), then the 12-17 V rail.
- Bulk caps on the 12-17 V rail: 1000 uF + 4 x 100 uF.
- Buck converter: 12-17 V to **+5V**. This feeds the servo power (PCA9685 V+) and, per the schematic, the Nano VIN.
- **LOGIC** net = the Nano's 5 V pin. It feeds the PCA9685 VCC, the motor driver +5V and the TMC2209 VIO.
- Encoder J1: pin 1 = 5V, 2 = GND, 3 = CH.A, 4 = CH.B. Its 5 V comes from the battery side, which explains the "battery off -> encoder drifts" gotcha.

**Old schematic's motor driver: IRF3205 module (DIR + PWM type). NOT what's on the robot.**
- Inputs: DIR1 (direction, high/low), PWM1 (speed duty), +5V (logic), GND. Channel 2 (DIR2/PWM2) is unused.
- Power: PWR = 12-17 V. Outputs 1A/1B go to the motor (DC_A/DC_B).
- (Superseded: the real driver uses both MA and MB; see above.)

**Schematic Nano pin map**
- D2 = DC_DIR, D3 = DC_PWM, D4 = CH.A, D5 = CH.B
- D6 = STP_EN1, D7 = STP_STEP1, D8 = STP_DIR1, D9 = STP_EN2, D10 = STP_STEP2, D11 = STP_DIR2
- A4/A5 = I2C (SDA/SCL) to the PCA9685
- Servos S1-S5 are on PCA9685 PWM0-4.
- Two TMC2209 stepper drivers (NEMA17) appear in the schematic. No current firmware uses them.

**Pin mismatch: RESOLVED for the outtake (14 Oct 2026).** The physical robot does **not** follow the schematic pin map. Real wiring: motor **MA=D8, MB=D9, PWM=D11**, encoder **D2/D3**. The motor runs with these; with 6/7/11 it got no power.
- On the real driver, D8 (MA) and D9 (MB) go to the two direction inputs and D11 to PWM. Which input is IN1 doesn't matter: direction is set by `OUTTAKE_MOTOR_DIRECTION`.
- Treat the schematic's Nano pin numbers as unreliable until someone checks them against the board. The schematic should be updated to match the real wiring.
- `Slide_Test` has 8/9/11 (updated by Tinzen). **`Robot_Controller_V2_1` still has 6/7/11 and must change to 8/9/11.** `Slide_Encoder_Check` (6/7/11) and `Motor_Pin_Test` (DIR 6, PWM 11) also still have the old pins.

**To check:** the schematic appears to put the buck +5V into the Nano **VIN**. VIN goes through the Nano's regulator, which needs about 7 V or more, so 5 V on VIN gives a sagging logic rail when USB is unplugged (i.e. at the competition). Feeding the buck 5 V into the Nano's 5V pin is the usual fix. Verify on the real board.
