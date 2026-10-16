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
4. **The Nano is tight: 30 KB flash, 2 KB RAM.** V2.1 was 28.9 KB / 1.16 KB (IDE) before the 16 Oct changes. avr-gcc here with a stand-in Encoder library: 28,594 B then, 27,210 B now (all-profiled servos removed the timed / instant code, `P` added). Read the IDE's number after uploading: that's the one that counts. Keep strings in `F()`, no new libraries, check the size after every change.
5. **Don't change tuned values unless asked**: servo poses, gains, Tinzen's choices (Slide_Test has no travel limit on purpose).
6. **V2.1 owns its classes.** `MotionProfile`, `ServoAxis`, `MotorController` in `Robot_Controller_V2_1/` are newer than the copies in `calibration/`. Don't copy between them; each sketch keeps its own.
7. **Sequence sync: whichever was edited last wins.** The sequence is `Robot_Controller_V2_1/Sequence.h`. `tools/sequence_editor.html` opens and saves that same file (Chrome / Edge, "Open robot folder"). Keep them in step:
   - **`Sequence.h` or `RobotConfig.h` edited by hand or by an agent** → refresh the editor's built-in snapshot (what it shows before a folder is opened), with this one-off from the repo root:
     ```
     python3 -c "import re,datetime;p='tools/sequence_editor.html';h=open(p,encoding='utf-8').read();[h:=re.sub(r'(<script type=\"text/plain\" id=\"%s\">).*?(</script>)'%t,lambda m,x=open(f,encoding='utf-8').read():m.group(1)+x+m.group(2),h,flags=re.S) for t,f in (('snap-seq','Robot_Controller_V2_1/Sequence.h'),('snap-cfg','Robot_Controller_V2_1/RobotConfig.h'))];h=re.sub(r'(id=\"snap-date\">).*?(</script>)',lambda m:m.group(1)+datetime.date.today().strftime('%d %b %Y')+m.group(2),h,flags=re.S);open(p,'w',encoding='utf-8').write(h)"
     ```
   - **Editor used without the folder** (Download / Copy) → the user gives you that text: write it to `Sequence.h`, then refresh the snapshot.
   - Keep the format the editor parses: `// ---- SECTION ----` headers, `const float NAME = value;  // note` lines, table rows `{ "NAME", arm, arm deg/s, scoop, scoop deg/s, slide, slide mm/s (a name), vibrate, WAIT_x(n) },  // note`. New angle names go in the ARM ANGLES / SCOOP ANGLES sections.
   - Step semantics live in the firmware's step runner and in the editor's `simulate()`. Change both together.
8. **Log every change** that affects behaviour, specs, pins or workflow: one dated line in the change log below.

## Workflows

**Slide gear ratio changes** (Tinzen may just say "step-up is now X:1"):
1. `ROBOT_SPECS.md` §3: step-up, counts per spool rev, mm per count, free speed, stall pull.
2. `RobotConfig.h`: `SLIDE_STEP_UP` only (mm/count and free speed are computed from it).
3. Calibration sketches: the `MM_PER_TICK` / `MM_PER_COUNT` lines in `calibration/Slide_Encoder_Check`, `Slide_Test`, `Slide_PID_Tune`.
4. `Sequence.h` does **not** change (mm and mm/s). Gains are in motor counts, so only `SLIDE_KG` / `SLIDE_KA` need a re-check on the robot; say so.
5. If the new free speed × 0.8 is below `SLIDE_FAST` or `SLIDE_RETRACT_SPEED` in `Sequence.h`, tell the user.
6. Refresh the editor snapshot (rule 7), compile V2.1, log it.

**Servo calibration results arrive** (three µs numbers from `Servo_Offset_Cal`): best done in the editor's **Calibrate servos** tab ("How the robot moves"); it can also find the numbers itself over the live link. By hand: compute each named angle's pulse widths with the old `*_CAL_US`, set the new `*_CAL_US`, then solve each angle back from the same pulse widths (arm: average of the R and L results), write both files, refresh the snapshot.

## Repo map

| Path | What it is |
|---|---|
| `ROBOT_SPECS.md` | All robot numbers, with status (confirmed / calc / CAD / TODO) and the constants that use them. |
| `Robot_Controller_V2_1/` | **The competition program.** `Sequence.h` = the state machine (one row per step), `RobotConfig.h` = hardware numbers, `.ino` = step runner + safety + telemetry + console. Its README explains every setting; `STATE_MACHINE.md` maps the table onto a classic switch FSM. |
| `calibration/` | Tuning sketches in order (README has pictures): `Slide_Encoder_Check`, `Slide_Test`, `Slide_PID_Tune`, `Servo_Offset_Cal`. `_old/Motor_Pin_Test` is retired. |
| `SEQUENCE_TUNING.md` | How to tune the sequence in the editor first, then confirm on the robot. |
| `tools/sequence_editor.html` | Edit the sequence with the 3D twin (play bar, Fusion-style camera, locked by default), Step & move tab (step menus, ↑ ↓ live nudging, next-step condition, ⏮ / ▶ / ⏭ for one step, run buttons), Pose sliders tab, target + speed menus in every table cell, draggable ◆ keyframes on the timeline (up/down = position, sideways = servo speed), undo/redo, predicted timing, live link (Connect in the top bar; Robot panel under the twin: Stop, Reset to start, Limp, **Robot follows timeline** = streams the twin's pose with `P`, so edits run without uploading), guided servo calibration. Needs `three.min.js` + `twin_model.js` beside it. Screenshots in `tools/img/`. |
| `tools/run_viewer.html` | Logged runs: plots, Gantt, FF fit, 3D twin. |
| `tools/capture_log.py` | Serial → `logs/run_*.csv` plus a `.html` with the run embedded. First-time setup (Python, pyserial, ports, troubleshooting): `Robot_Controller_V2_1/LOGGING.md`. |
| `tools/cad_twin/` | STEP → `twin_model.js` pipeline. Mesh IDs change on every re-export. |
| `tools/sil_sim/` | PC build of V2.1 against mocks plus a slide plant model. |
| `Robot_Controller/`, `Robot_Controller_V2/` | V1 and V2 programs. Reference only. `Version 1 Demonstration.mp4` shows the V1 sequence V2.1 copies for the transfer. |
| `logs/` | Captured runs. |

## Status

- V2.1 table-driven sequence: full runs on the robot 15 Oct (about 12.8–15 s). Slide gearing now 1:1 (16 Oct).
- **Next on the robot:**
  - servo calibration: editor → Calibrate servos (or `Servo_Offset_Cal`), so the twin angles become real;
  - confirm slide motor direction (`calibration/Slide_Encoder_Check`);
  - **`X_TIP` (1788.9 mm) is past the end stop (~1694 mm)**: every 15 Oct run stalled in CREEP and the fallback skipped TIP_HOLD. Set it just before the stop;
  - tune `S_CARRY`, `S_POUR`, `PICKUP` time; slide gains;
  - measure the real slide angle and robot height.
- See ROBOT_SPECS.md §7 for the full open list.

## Known mismatches (fix when you touch the file)

- `calibration/Slide_Test` and `Slide_PID_Tune` print paste lines named `OUTTAKE_K*`. V2.1 now calls them `SLIDE_K*` (same units).
- `Robot_Controller_V2`: pins 6/7/11, encoder direction +1. Reference only; don't fix it.
- Twin: at `A_START` the CAD scoop still overlaps the bucket a little. The servo calibration (or the scale check) settles it.
- Servo calibration done 16 Oct (R 910 / L 2105 / scoop 1968 µs). It confirmed the 15 Oct +4.4° arm tweaks were compensating the old calibration: re-worked, `A_SCOOP` is 110.3° (CAD floor pose 110.6°). `S_DOWN` (−259.6°) is now 0.4° inside the scoop soft limit (−260°).
- Logs recorded before 16 Oct carry `OUTTAKE_MM_PER_TICK` 0.5317, so their replays show the slides 1.8× too far out.

## Gotchas

- **Battery ON for any encoder or motor test.** On USB alone the encoder count drifts.
- **Opening the serial port resets the Nano.** V2.1 then waits 150 ms (`BENCH_WINDOW_MS`) for `b`: the sequence editor sends it (Connect → Hold still), so the robot holds still. Anything else that opens the port (Arduino Serial Monitor, `capture_log.py`) doesn't, so the robot starts its run: use `WAIT_FOR_GO = true` for bench logging, never at the competition.
- The encoder must stay on D2/D3 (the only interrupt pins).
- **The KiCad schematic is out of date.** Trust ROBOT_SPECS.md.
- Hobby servos can't report position: "arrived" for a servo move is an estimate.
- **Robot follows timeline ≠ the uploaded run.** It streams the editor's simulated pose (`P`, ~25/s): the editor's timing, servos at native speed toward each streamed angle, slides re-planned toward each streamed mm. No stall fallback, no WAIT_ARRIVED on the real slides. Always confirm with **Run all** after uploading.

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
  - New `Robot_Controller_V2_1/LOGGING.md` (first-time Python + pyserial logging guide); linked from the V2.1, calibration, Slide_Test, Slide_PID_Tune and logs READMEs.
  - `calibration/Servo_Offset_Cal`: new README. `p` in level mode now prints the level pulses separately and still pastes the reference values; typing `level` twice no longer offsets again from the level pulses.
- **16 Oct 2026**:
  - Slide gearing 1.8:1 → **1:1** (hardware swapped). `SLIDE_STEP_UP` 1.0, mm/count 0.2954, free speed ~824 mm/s at 12 V. The 15 Oct values were tuned on the robot at the wrong scale, so Tinzen asked to keep the motion: every slide mm value ÷ 1.8 (same counts): `X_CREEP` 3000 → 1666.7, `X_TIP` 3220 → 1788.9, `SLIDE_FAST` 1000 → 555.6, `SLIDE_CREEP` 200 → 111.1, `SLIDE_MAX` 2000 → 1111.1, `SLIDE_ACCEL` 5000 → 2777.8, tolerances 1.5/12/80 → 0.83/6.7/44.4 mm. Gains unchanged (counts). Calibration sketches: mm/count 0.2954, their default speeds / settle band scaled the same way. `ServoTesting/RobotConfig.h` kept identical to V2_1's.
  - Editor: snapshot refreshed; fallback `CFG_DEFAULT` now matches `RobotConfig.h` (was stale calibration 855.2/2144.8/1837.3 and 1.8).
  - Max travel measured from the 15 Oct logs: end stop ~1694 mm (~5735 counts).
  - New `Robot_Controller_V2_1/STATE_MACHINE.md`.
  - Servo calibration from the editor (Calibrate servos): `*_CAL_US` 940/2060/1900 → **910/2105/1968**. Applied with "How the robot moves": every arm angle −4.6/−4.7°, every scoop angle −10.2° (e.g. A_SCOOP 115 → 110.3, S_SCOOP −69.4 → −79.6), same pulse widths as before. `ServoTesting/RobotConfig.h` copied.
  - Editor live link fix: Hold still no longer depends on catching the `#FW` start-up line (a reset glitch garbled it, so the robot ran its sequence and the editor never became ready). It now sends `b` every 50 ms for up to 4 s after connecting until `#MODE,1`; `#MODE,1` alone marks the robot ready; `#FW`/`#STATES` are found even with junk before them; missing step names are fetched with `l`.
  - V2.1 bench handshake: at power-up the robot waits 150 ms for `b` from a computer, then runs as normal. `b` also works any time (stops a run, waits for commands). Jog commands are accepted after a finished run. New servo calibration commands `U` (raw pulse), `L` (limp), `M` (back to degrees at the reference pose). `RUN_MODE` no longer needs changing for the bench. Flash 28.9 KB. SIL mock: `Serial.flush()`, time passes in `setup()` busy-waits.
  - Sequence editor usability pass: Connect & move tab (hold still / run on connect, status badge, plain button names, Save pose), Calibrate servos tab (guided calibration over the live link, preview of what Apply changes, Apply never moves the robot), + Add step button / menu (also under the table and in the Step tab), clearer file status.
  - Tinzen's sequence edits (saved from the editor): new `IDLE` first row (A_START / S_START, INSTANT, WAIT_ARRIVED(0)); `A_START` −52.1 → −31.6°, `S_START` −19.6 → 5.4°; `X_CREEP` 1666.7 → 1670.2; VIBRATE 1000 → 500 ms; EXTEND also takes the scoop to `S_DOWN`; TIP_HOLD 1000 → 1500 ms; RETRACT goes to 200 mm (was 0); unused `X_NEW1` = 200 added. Snapshot refreshed. Note: `#STATES` now lists IDLE twice (state 0 = not running, state 1 = the IDLE row).
  - V2.1 new console command `P <arm> <scoop> [mm] [mm/s] [vib]` (follow the editor's timeline): servos `jumpTo` the angles (leaves calibration/limp at that pose; the editor waits out the servo travel itself, ~430 °/s), slides `setTargetBlend` toward mm at SLIDE_MAX/SLIDE_ACCEL keeping the current profile speed and arriving at mm/s (mm < 0 = slides untouched, re-plans only when the target moves > 1 count), vib > 0 shakes. No reply line. `cmdBuf` 28 → 40. SIL-tested (streamed ramp: slide setpoint ~35 mm / ~70 ms behind at 500 mm/s, ends 2 mm short) and compiled with avr-gcc 7.3 / core 1.8.6 + a stand-in Encoder: +576 B flash, +12 B RAM (`setTargetBlend` is now a shared function). Check the IDE's number.
  - Sequence editor rework (Tinzen's list): Open file removed, the folder button shows "✓ Folder opened: <name>"; Connect robot in the top bar; Step and Connect & move tabs merged into **Step & move** (step editor, pose sliders with load-a-name menus + per-row Save, run buttons); permanent **Robot** panel under the twin (■ Stop, ↺ Reset to start = `x`, `A A_START`, `S S_START`, `o 0 300`; Limp toggle = `x` + `L 0`, click again = `M` at the twin's pose; Robot follows timeline / Slides follow too; console + command box); toggles fill when on; **Lock view** on at every start; table cells are menus (named values with numbers, common move times, "type a number…"); **Show this step** fixed (it jumped to the step's start pose, and the live robot frame overrode the twin while connected): it now plays the step and holds its end pose, clicking a step shows its end pose, and the twin always shows the timeline (ghost = real robot) except while the robot runs its uploaded sequence; ◆ keyframes at the end of every move, drag up/down = value (names change everywhere), sideways = a timed move's ms, Shift = fine; undo/redo (Ctrl+Z); named values can be renamed and deleted (not the ones the firmware uses, not while a step uses them); `writeHeader` rewritten line by line to support that. Tuning guide and screenshots updated.
  - **Motion model rework (Tinzen):** every move is a trapezoid profile. `Sequence.h` columns are now `{ NAME, ARM, deg/s, SCOOP, deg/s, SLIDE, SLIDE mm/s, VIBRATE, NEXT WHEN }`: MOVE (INSTANT / PROFILED / timed ms) and ARM VIA removed, a speed per servo move (0 = `ARM_SPEED` / `SCOOP_SPEED`), one accel per servo. A new target takes over a running servo move smoothly (`MotionProfile::moveFrom`: carries the speed in, or brakes at the accel first if the new target is behind); HOLD lets it run on. INSTANT rows became `ARM_FAST` 358.3 / `SCOOP_FAST` 430 °/s (the servos' own top speed) with the existing accels, so those moves are a little softer than before. SWEEP split into `SWEEP_UP` (arm → A_PEAK 30.6 °/s, scoop → S_SCOOP 91.9 °/s, WAIT_MS(1000)) + `SWEEP_DOWN` (arm → A_SCOOP 41.1 °/s, WAIT_ARRIVED); LIFT 1000 ms timed → 177.8 °/s (same ~1 s). Slide speeds are named only (`SLIDE_FAST`, `SLIDE_CREEP`, `SLIDE_RETRACT_SPEED`); `SLIDE_MAX` → `SLIDE_RETRACT_SPEED` 1111.1 with new `SLIDE_RETRACT_ACCEL` (2777.8 = unchanged, raise it to save time). `TIMED_MOVE_EASE` removed. Tinzen's unsaved WAIT row (500 ms after SWEEP) added. Firmware: TimedMove / instantTo / native-speed code removed, ServoAxis always profiles. SIL run OK (14.4 s); flash 27,210 B (avr-gcc, stand-in Encoder).
  - Editor for the new model: table cells = target + speed menu (slide speeds names only, servo speeds capped at the servo's top speed, over-speed flagged); What happens / Note / Move / Arm via / Vibrate columns gone (shake is in the Step & move tab); ◆ sideways = that servo move's speed (solved from its real start: position, speed, accel), slides vertical only; Step & move tab: ↑ ↓ value boxes nudge a pose live (name = everywhere; twin + robot follow; one undo step per run of presses), next-step menu, ⏮ Step start (robot lines up smoothly with `A`/`S`/`o`, no jump) / ▶ Play step (only that step) / ⏭ Step end; one playback speed (1× / 0.5× / 0.25× / 0.1×) also sent to the robot as `k`; pose sliders moved to their own tab; twin smaller, console shorter, tabs panel bottom aligned with the robot panel. Old-format `Sequence.h` is refused with a message.
  - Editor fixes (Tinzen's report): (1) nudging A_PEAK looked dead: the twin jumped to when ALL of SWEEP_UP's moves end (3.0 s, scoop), by which time SWEEP_DOWN had already moved the arm on. Selecting a step / nudging / dragging a ◆ now shows the targets (solid) with the run as a grey ghost, and a note says when a move is cut short or has moved on; the view follows the selected step / move through any edit (`S.focus`). (2) Scoop "vibrating" while nudging LIFT: the end time landed in VIBRATE and a still frame drew the shake offset (and Robot follows timeline sent the shake). Still frames now show the scoop without the shake and the follow stream only shakes while playing. (3) Timeline squashed on mouse-down when dragging a ◆ or an edge (it re-scaled to 112 % of the run, and live telemetry redrew it at once): the scale now stays exactly as it is during a drag. ▶ Play step now plays the step's own time (until the next step starts), ⏭ shows its targets. Sweep speeds re-solved from how the arm really enters SWEEP_UP (still moving from ARM_DOWN): SWEEP_UP arm 30.6 → 35 °/s (reaches A_PEAK at 0.98 s; at 30.6 it never got there), SWEEP_DOWN 41.1 → 42.4 °/s. SIL agrees (A_PEAK at 2.16 s, SWEEP_DOWN at 2.18 s).
  - Per-axis "arrived" guards (Tinzen): `WAIT_ARM_ARRIVED(ms)`, `WAIT_SCOOP_ARRIVED(ms)`, `WAIT_SLIDE_ARRIVED(ms)` (that one axis's move finished, even if an earlier row started it) and `WAIT_SCOOP(%)`; `WAIT_ARRIVED` = all this row moved, as before. `WAIT_SLIDE_ARRIVED` also lets the slide blend into the next row. Firmware +82 B (27,292 B avr-gcc). SIL: SWEEP_UP on `WAIT_ARM_ARRIVED(0)` hands over at 0.97 s, as the editor predicts. Switching guard in the editor resets the number sensibly (arrived → 0 ms, % → 100, after ms → the step's current length). Timeline ◆ drag freely in both directions (position and arrival time / speed at once, 3 px dead zone per direction); click a ◆ to select it, then ↑ ↓ position, ← → arrival time (10 ms), Shift ×10, Alt ×0.2, Esc. "Play the edited sequence on the robot" renamed "Play from this step to the end, on the robot".
