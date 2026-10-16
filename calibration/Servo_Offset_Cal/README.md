# Servo_Offset_Cal

Finds the three servo pulse widths (right arm, left arm, scoop) at a known pose. With them, the angles in `Sequence.h` and the sequence editor's twin are real degrees.

> **Easier:** with `Robot_Controller_V2_1` uploaded, the sequence editor's **Calibrate servos** tab does the same procedure with buttons over the live link (Connect & move → Hold still → Connect robot). Use this sketch when the editor isn't an option.

**Why it's needed:** hobby servos have no position feedback. The code turns an angle into a pulse width with:

```
pulse (µs) = CAL_US + DIR × US_PER_DEG × (angle − CAL_DEG)
```

`US_PER_DEG` and `DIR` come from the servo and gearing. `CAL_DEG` is the reference pose from CAD. `CAL_US` is the only unknown, and it changes whenever a servo horn or gear is re-fitted. This sketch measures it.

## The reference pose

**Scoop laid flat on the intake floor.** It's a hard stop: the floor fixes the scoop's angle, and the scoop resting on the floor fixes the arm's angle, so one pose sets all three servos. CAD: arm **110.6°** (forward of vertical), scoop **−69.4°** (relative to the arm).

<table><tr>
<td><img src="../img/ref_pose_side.png" width="360" alt="Twin side view: arm reaching forward and down, scoop blade flat on the floor"></td>
<td><img src="../img/ref_pose_3q.png" width="360" alt="Twin three-quarter view of the scoop lying flat in the intake area"></td>
</tr><tr><td colspan="2">The whole blade sits on the floor; the curled back of the scoop points up. This is also the end of the SWEEP step (A_SCOOP / S_SCOOP).</td></tr></table>

## Setup

- Robot on a flat surface at intake-floor height, **slides fully in**, **battery on** (the PCA9685 servo power comes from the battery side).
- A strip of paper, and a phone level if you'll do the scale check.
- Upload `Servo_Offset_Cal.ino`, open the Serial Monitor at **115200**, line ending **Newline**.

The servos start **limp** (no pulse) and stay limp until you type `cal`, so nothing jumps at power-on. The starting pulses are the current `RobotConfig.h` values (940 / 2060 / 1900 µs), so `cal` should already be close.

## Commands

| Command | Does |
|---|---|
| `cal` | All three servos drive to the reference pose with the current numbers. Also returns from `level`. |
| `r <µs>` `l <µs>` `s <µs>` | Set the right-arm / left-arm / scoop pulse directly (500–2500). |
| `r+` `r-` `l+` `l-` `s+` `s-` | Nudge one servo by the step size. |
| `a+` `a-` | Nudge **both** arm servos together, in opposite pulse directions so they don't fight. `a+` = arm further out / down. |
| `step <µs>` | Nudge size. Default 5; use 2 for the final touch. |
| `off r` · `off l` · `off s` · `off all` | Make that servo limp (no pulse). |
| `on r` · `on l` · `on s` · `on all` | Drive it again at its current pulse. |
| `level` | Scale check pose: arm vertical, scoop blade level (see below). |
| `p` | Print the three lines to paste. |
| `?` | Help. |

After every change it prints the current pulses, e.g. `R 940   L 2060 (limp)   S 1900 us`.

Pulse direction, as configured in `RobotConfig.h`: **`r+` raises the arm** (right servo), **`l+` lowers it** (left servo). If a servo goes the other way, the `ARM_*_DIR` sign is wrong: tell your agent before going further. The scoop direction is easiest to learn by trying `s+` once.

## Procedure

1. **Lower the scoop to the floor by hand** while the servos are limp, so nothing jumps far when they switch on.
2. **`cal`.** All three servos drive to the reference pose. Watch for a servo straining or a big jump; if it does, type `off all` and check the horns.
3. **Arm limp, set the scoop.** Type `off r` then `off l`. The arm settles until the scoop rests on the floor under its own weight (press it down lightly if the servo gears hold it up). Use `s+` / `s-` until the blade lies **flat**: the paper strip is trapped under the lip **and** under the back edge of the blade.
4. **Right arm servo.** `on r`. If the arm lifts or presses down, adjust with `r+` / `r-` until switching `on r` / `off r` doesn't move the arm and the paper stays trapped at both edges. Then `off r`.
5. **Left arm servo.** `on l`, same as step 4 with `l+` / `l-`.
6. **`on all`.** The arm should hold quietly. If it hums or the current draw rises, the two arm servos disagree: nudge `l` by ±5 µs (or `step 2`) until it stops.
7. **`p`.** It prints:
   ```
   const float ARM_R_CAL_US = 940.0f;
   const float ARM_L_CAL_US = 2060.0f;
   const float SCOOP_CAL_US = 1900.0f;
   ```

## Using the numbers

**Best: the sequence editor.** `tools/sequence_editor.html` → Open robot folder → **Calibrate servos** tab → **Use the new numbers**:

1. Type the three numbers.
2. Under **What should stay the same?** choose **How the robot moves** (pulse widths stay the same; the angles in `Sequence.h` are re-worked so the twin matches the robot). Use this when the robot already moves correctly. The other choice, **The angle numbers**, moves the robot to match the angles in `Sequence.h`. The box underneath lists exactly what will change.
3. **Apply to the editor** (changes numbers only: it doesn't move the robot) → **Save** → upload `Robot_Controller_V2_1`.

**By hand:** paste the three lines into `Robot_Controller_V2_1/RobotConfig.h` and ask your agent to re-work the angles in `Sequence.h` so the pulse widths stay the same (AGENTS.md, "Servo calibration results arrive"). Pasting alone, without re-working the angles, changes how the robot moves.

Also update the starting guesses at the top of this sketch (`armRUs`, `armLUs`, `scoopUs`) so the next `cal` starts from the new numbers.

## Optional scale check (2 minutes)

Checks the servos really are 300° (**8.0 µs per arm degree** through the 40T:48T gears, **6.667 µs per scoop degree**). Do it after step 7.

1. **`level`.** The arm should stand vertical and the scoop blade should be level on top:

   <img src="../img/level_pose_side.png" width="380" alt="Twin side view: arm posts vertical, scoop blade horizontal on top">

2. Put a phone level on an arm post and use `a+` / `a-` until it reads 90°. Put it on the blade and use `s+` / `s-` until it's level.
3. **`p`.** In level mode the first line is the level pulses (`# LEVEL pose … R … S …`). The paste lines below it are still the reference values.
4. Editor → Calibrate servos → **Scale check**: enter the level R and S pulses → **Work out scale**. More than about 3% off 8.0 / 6.667 means the servo scale in `RobotConfig.h` needs changing: tell your agent.
5. `cal` goes back to the reference pose; nudges made in level mode are discarded.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `# !! no PCA9685 on I2C` | Battery off, or SDA/SCL/power to the servo board not connected. Address must be 0x40. |
| Servos don't move after `cal` | Battery off (the board's logic may be powered from USB but the servos aren't). |
| A servo jumps a long way on `cal` | The horn or gear was re-fitted a tooth off. Fine: tune it back with this procedure. If it hits a hard stop, `off all` at once. |
| Arm hums at the end | The two arm servos fight. Nudge `l` in small steps until it's quiet. |
| `# pulse must be 500-2500` | The value given with `r` / `l` / `s` is out of range. |
| Nudge goes the wrong way | The direction sign in `RobotConfig.h` (`ARM_R_DIR`, `ARM_L_DIR`, `SCOOP_DIR`) doesn't match the robot. Tell your agent. |

## Keep in sync

The top of `Servo_Offset_Cal.ino` holds a copy of the servo section of `RobotConfig.h`: outputs, `CAL_ARM_DEG` / `CAL_SCOOP_DEG`, µs per degree, directions. If any of those change in `RobotConfig.h` (or `ROBOT_SPECS.md` §4), change them here too.
