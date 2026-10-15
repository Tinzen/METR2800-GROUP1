# AGENTS.md: read this first

This is the entry point for every agent (Claude, Codex, teammates' tools) and every person working in this repo. Keep it short. Robot numbers go in **`ROBOT_SPECS.md`**, not here.

## Rules

1. **`ROBOT_SPECS.md` is the source of truth for robot numbers** (pins, ratios, spool, mm/count, directions, servo calibration, geometry). In V2.1 code they live in `Robot_Controller_V2_1/RobotConfig.h`. When a measurement changes:
   - update `ROBOT_SPECS.md` first;
   - then update the constants named in its **Code** column;
   - then add a line to the change log below.
2. **Quick spec check, only for the file you are editing.** Before you upload or commit a change to a sketch, run one grep and compare against ROBOT_SPECS.md §1 (pins), §3 (slides) and §4 (servos):
   ```
   grep -nE "OUTTAKE_(MA|MB|PWM|ENC_A|ENC_B)|SLIDE_(ENCODER_DIR|MOTOR_DIR|STEP_UP|SPOOL_DIA_MM|COUNTS_PER_MOTOR_REV)|ENCODER_DIRECTION|MOTOR_DIRECTION|MM_PER_(TICK|COUNT)|_CAL_US|_CAL_DEG|US_PER_DEG|MOTOR_(A|B|PWM) *=" <file>
   ```
   - Don't audit other sketches or re-derive numbers already in the spec file.
   - If a file you are **not** editing is wrong, add it to "Known mismatches" instead of fixing it.
3. **Others edit this repo at the same time.** Re-read a file right before you edit it. Edit in place; never paste a whole file from memory. Don't overwrite files you didn't change.
4. **The Nano is tight: 30 KB flash, 2 KB RAM.** V2.1 is at 28.0 KB / 1.16 KB. Keep strings in `F()`, no new libraries, check the size after every change.
5. **Don't change tuned values unless asked**: servo poses, gains, Tinzen's choices (Slide_Test has no travel limit on purpose).
6. **V2.1 owns its classes.** `MotionProfile`, `ServoAxis`, `MotorController` in `Robot_Controller_V2_1/` are newer than the copies in `calibration/`. Don't copy between them; each sketch keeps its own.
7. **Sequence sync: whichever was edited last wins.** The sequence is `Robot_Controller_V2_1/Sequence.h`. `tools/sequence_editor.html` opens and saves that same file (Chrome / Edge, "Open robot folder"). Keep them in step:
   - **`Sequence.h` or `RobotConfig.h` edited by hand or by an agent** → refresh the editor's built-in snapshot (what it shows before a folder is opened), with this one-off from the repo root:
     ```
     python3 -c "import re,datetime;p='tools/sequence_editor.html';h=open(p,encoding='utf-8').read();[h:=re.sub(r'(<script type=\"text/plain\" id=\"%s\">).*?(</script>)'%t,lambda m,x=open(f,encoding='utf-8').read():m.group(1)+x+m.group(2),h,flags=re.S) for t,f in (('snap-seq','Robot_Controller_V2_1/Sequence.h'),('snap-cfg','Robot_Controller_V2_1/RobotConfig.h'))];h=re.sub(r'(id=\"snap-date\">).*?(</script>)',lambda m:m.group(1)+datetime.date.today().strftime('%d %b %Y')+m.group(2),h,flags=re.S);open(p,'w',encoding='utf-8').write(h)"
     ```
   - **Editor used without the folder** (Download / Copy) → the user gives you that text: write it to `Sequence.h`, then refresh the snapshot.
   - Keep the format the editor parses: `// ---- SECTION ----` headers, `const float NAME = value;  // note` lines, table rows `{ "NAME", arm, scoop, slide, mm/s, move, via, vibrate, WAIT_x(n) },  // note`. New angle names go in the ARM ANGLES / SCOOP ANGLES sections.
   - Step semantics live in the firmware's step runner and in the editor's `simulate()`. Change both together.
8. **Log every change** that affects behaviour, specs, pins or workflow: one dated line in the change log below.

## Workflows

**Slide gear ratio changes** (Tinzen may just say "step-up is now X:1"):
1. `ROBOT_SPECS.md` §3: step-up, counts per spool rev, mm per count, free speed, stall pull.
2. `RobotConfig.h`: `SLIDE_STEP_UP` only (mm/count and free speed are computed from it).
3. Calibration sketches: the `MM_PER_TICK` / `MM_PER_COUNT` lines in `calibration/Slide_Encoder_Check`, `Slide_Test`, `Slide_PID_Tune`.
4. `Sequence.h` does **not** change (mm and mm/s). Gains are in motor counts, so only `SLIDE_KG` / `SLIDE_KA` need a re-check on the robot; say so.
5. If the new free speed × 0.8 is below `SLIDE_FAST` or `SLIDE_MAX` in `Sequence.h`, tell the user.
6. Refresh the editor snapshot (rule 7), compile V2.1, log it.

**Servo calibration results arrive** (three µs numbers from `Servo_Offset_Cal`): best done in the editor's Calibration tab ("Keep the robot's motion"). By hand: compute each named angle's pulse widths with the old `*_CAL_US`, set the new `*_CAL_US`, then solve each angle back from the same pulse widths (arm: average of the R and L results), write both files, refresh the snapshot.

## Repo map

| Path | What it is |
|---|---|
| `ROBOT_SPECS.md` | All robot numbers, with status (confirmed / calc / CAD / TODO) and the constants that use them. |
| `Robot_Controller_V2_1/` | **The competition program.** `Sequence.h` = the state machine (one row per step), `RobotConfig.h` = hardware numbers, `.ino` = step runner + safety + telemetry + console. Its README explains every setting. |
| `calibration/` | Tuning sketches in order (README has pictures): `Slide_Encoder_Check`, `Slide_Test`, `Slide_PID_Tune`, `Servo_Offset_Cal`. `_old/Motor_Pin_Test` is retired. |
| `SEQUENCE_TUNING.md` | How to tune the sequence in the editor first, then confirm on the robot. |
| `tools/sequence_editor.html` | Edit the sequence with the 3D twin (play bar, Fusion-style camera), plain-English Step tab, offline jog + capture, predicted timing, live link to the robot (Web Serial), servo calibration. Needs `three.min.js` + `twin_model.js` beside it. Screenshots in `tools/img/`. |
| `tools/run_viewer.html` | Logged runs: plots, Gantt, FF fit, 3D twin. |
| `tools/capture_log.py` | Serial → `logs/run_*.csv` plus a `.html` with the run embedded. |
| `tools/cad_twin/` | STEP → `twin_model.js` pipeline. Mesh IDs change on every re-export. |
| `tools/sil_sim/` | PC build of V2.1 against mocks plus a slide plant model. |
| `Robot_Controller/`, `Robot_Controller_V2/` | V1 and V2 programs. Reference only. `Version 1 Demonstration.mp4` shows the V1 sequence V2.1 copies for the transfer. |
| `logs/` | Captured runs. |

## Status

- V2.1 rewritten as a table-driven sequence (15 Oct). Compiles, passes the SIL run. **Not run on the robot yet.**
- **Next on the robot:**
  - `Servo_Offset_Cal`, then the editor's Calibration tab (twin angles become real);
  - confirm slide motor direction (`calibration/Slide_Encoder_Check`);
  - measure `X_CREEP` / `X_TIP` (placeholders 450 / 530 mm; CAD says ~1650 / 1740);
  - tune `S_CARRY`, `S_POUR`, `PICKUP` time; slide gains;
  - measure the real slide angle and robot height.
- See ROBOT_SPECS.md §7 for the full open list.

## Known mismatches (fix when you touch the file)

- `calibration/Slide_Test` and `Slide_PID_Tune` print paste lines named `OUTTAKE_K*`. V2.1 now calls them `SLIDE_K*` (same units).
- `Robot_Controller_V2`: pins 6/7/11, encoder direction +1. Reference only; don't fix it.
- Twin: at `A_START` the CAD scoop still overlaps the bucket a little. The servo calibration (or the scale check) settles it.

## Gotchas

- **Battery ON for any encoder or motor test.** On USB alone the encoder count drifts.
- **Opening the serial port resets the Nano.** In `RUN_FULL` with `WAIT_FOR_GO = false` that starts a run. Use `RUN_CONSOLE` or `WAIT_FOR_GO = true` on the bench, never at the competition.
- The encoder must stay on D2/D3 (the only interrupt pins).
- **The KiCad schematic is out of date.** Trust ROBOT_SPECS.md.
- Hobby servos can't report position: "arrived" for a servo move is an estimate.

## Change log

- **14 Oct 2026**:
  - Pins confirmed as MA 8 / MB 9 / PWM 11, encoder on 2/3; the driver is IN1/IN2 + PWM.
  - Encoder direction −1 (Slide_Test). Spool 36.15 mm with 1:1 stringing, so 0.5317 mm/count.
  - Slide_Test travel limit removed (Tinzen). Slide_PID_Tune added. Battery-off encoder drift found.
- **15 Oct 2026 (morning)**:
  - AGENTS.md rewritten as the entry point; numbers moved to ROBOT_SPECS.md; CLAUDE.md points here.
  - V2.1 pins 6/7 → 8/9, encoder direction −1, mm/count 0.5317. Slide_Encoder_Check pins 6/7 → 8/9.
  - Twin: motor-end bracket fixed, stray standoff removed (CAD footprint 393 × 370 mm).
- **15 Oct 2026 (afternoon)**:
  - V2.1 rewritten: sequence table in `Sequence.h` (V2 intake, V1 transfer with carry/pour/vibrate, slides fast → creep blend → tip hold → retract), hardware in `RobotConfig.h`, servos in degrees with a 3-number reference-pose calibration, step mode, slow mode, live-link console commands, RETRACT fallback on a slide stall. Adafruit servo library replaced by a small PCA9685 driver (same 50 Hz setup) to fit in flash.
  - Twin scoop direction flipped (+1): with it the scoop holds rocks at SCOOP/PICKUP and pours at TRANSFER. Reference pose from CAD: arm 110.6°, scoop −69.4°.
  - New `tools/sequence_editor.html`; `calibration/` folder (Slide_Encoder_Check, Slide_Test, Slide_PID_Tune moved in; new Servo_Offset_Cal; Motor_Pin_Test retired to `_old`).
  - `tools/sil_sim` updated for the new firmware.
- **15 Oct 2026 (evening)**:
  - Servo calibration set from Tinzen: the V2 SCOOP pose (R 940 / L 2060 / scoop 1900 µs) lies flat on the floor, so `*_CAL_US` = 940 / 2060 / 1900 and every angle in `Sequence.h` re-worked with the same pulse widths (A_SCOOP 110.6°, S_SCOOP −69.4° = the reference pose). Soft limits −55…125° / −260…20°. Robot motion unchanged.
  - Sequence editor: play bar on the twin, Fusion 360 / Inventor camera, Step tab (plain-English step editor), "What happens" column, add-step presets, jog sliders that pose the twin with no robot and capture arm, scoop and slide values. New `SEQUENCE_TUNING.md`.
  - Sequence editor: drag timeline edges to retime a step, drag rows to reorder, spill warning (scoop pours while loaded before the arm is over the bucket), before/after ghost when editing or jogging an angle. The twin flags a ~0.1 s spill risk at the LIFT → CARRY hand-over (arm ~30°); check on the robot, values not changed.
  - `tools/run_viewer.html`: same Fusion 360 / Inventor camera as the editor (patched in place).
