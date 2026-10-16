# Tuning the sequence digitally

Get the whole sequence right in the editor first. The robot is then only for confirming it. The tool is `tools/sequence_editor.html`: Chrome or Edge, with `three.min.js` and `twin_model.js` next to it. It reads and writes `Robot_Controller_V2_1/Sequence.h`, the same file the robot runs.

<img src="tools/img/tuning_overview.png" width="900" alt="Sequence editor: folder, undo, save and Connect robot buttons along the top; the 3D twin with a play bar and the Robot panel (Stop, Reset, Limp, Robot follows timeline, messages) on the left; the Step and move tab on the right">

## 0. How far you can trust the twin

| The twin is right about | The twin can't tell you |
|---|---|
| Order and overlap of steps, every motion profile (servos and slides), smooth take-overs, slide blending, predicted run time | Whether a loaded servo keeps up: a profile faster than about 430 °/s (358 °/s for the arm) finishes before the servo arrives (the editor flags those speeds) |
| Whether the scoop holds, is level or pours (the coloured strip) | Slide lag and stalls: the sim follows the profile perfectly, the real slides lag 0.1 to 0.2 s |
| Collisions you can see, once the angles are calibrated | Rocks |

- **Servo angles are only as good as the calibration.** Calibrated 16 Oct with the **Calibrate servos** tab (910 / 2105 / 1968 µs). Redo it if a servo horn is taken off or the twin and the robot disagree.
- **Slide tilt** (box above the twin): 14.9° is a placeholder. Measure the real angle and type it in. It only changes the picture.
- **Slide positions** `X_CREEP` / `X_TIP` are 1670.2 / 1788.9 mm (tuned on the robot 15 Oct, re-scaled to 1:1 gearing 16 Oct). `X_TIP` is past the end stop (~1694 mm): CREEP stalls and the RETRACT fallback skips TIP_HOLD. Set it just before the stop with `calibration/Slide_Test`. The CAD says about 1650 / 1740.

## 1. Open the robot folder

Click **Open robot folder** and pick `METR2800-GROUP1` (or `Robot_Controller_V2_1`). The button then reads **✓ Folder opened: METR2800-GROUP1**; click it again to switch folders. Until a folder is open you're on a read-only built-in copy and **Save** is greyed out. **Save** (Ctrl+S) writes `Sequence.h`, and `RobotConfig.h` too if you changed the calibration. Chrome remembers the folder: next time the button says **Reopen METR2800-GROUP1**.

**↶ ↷** (Ctrl+Z / Ctrl+Shift+Z) undo and redo any edit: table, Step & move tab, values, timeline drags.

## 2. Watch the sequence

- **Play**: the button on the twin, or Space. **‹ ›** (or ← →) jump one step. 0.25× is good for fast steps.
- **Timeline**: step blocks on top (click one to select it). Under them is the scoop strip: **green** holds rocks, **yellow** level, **red** pours. Then the traces: arm (blue), scoop (orange), slides (green, right axis).
- The **grey ghost** in the twin is where the arm and scoop are heading (the target). The solid model is where they are now. While you edit or jog an angle, the ghost shows the old value instead (before / after). With the robot connected, the ghost is the **real robot**.
- **Click a step** (table or timeline) and the twin jumps to where that step **ends**. In the Step & move tab, **⏮ Step start** goes back to where it begins (where the step before left things), **▶ Play step** plays only that step, **⏭ Step end** jumps to its end pose. The **speed** menu next to them (1× / 0.5× / 0.25× / 0.1×) is the same one as on the twin and the timeline, and it's also sent to the robot.

3D view, same controls as Fusion 360 / Inventor. It starts **locked** (the **View locked** button above it) so a stray drag or scroll can't spin it; click the button to unlock:

| Do | Mouse | Trackpad |
|---|---|---|
| Zoom | scroll, zooms at the cursor | two-finger scroll or pinch |
| Pan | middle-drag | Shift + drag |
| Orbit | Shift + middle-drag, about the point under the cursor | drag |
| Fit the robot | double-click (middle or left) | double-tap |

Scroll the wrong way round for you? **Reverse zoom** under the twin. The **View** menu jumps to Robot / Side / Field (works while locked).

## 3. Change what a step does: the Step & move tab

Click a step's **number** or **description** in the table (or its block on the timeline). The **Step & move** tab opens. It says in plain words what the step does, when it starts and how long it takes.

<img src="tools/img/tuning_step_tab.png" width="460" alt="Step and move tab for SWEEP_UP: summary, step start / play step / step end buttons with a speed menu, arm, scoop and slide target menus with value boxes and speed menus, shake and next-step menus">

| Control | What it means |
|---|---|
| **Arm / Scoop** | A named angle, a number, or **HOLD**. HOLD doesn't stop the axis: a move started in an earlier step keeps going. That's how servos run asynchronously: start a move, then change step on a timer while it finishes. |
| **value box** | The number the target stands for. **↑ ↓** nudge it live (0.5°, Shift ×10, Alt ×0.2; 1 mm for slides). If the target is a name, every step using it changes (the line under it says which). The twin jumps to the step's end pose, and a connected robot goes there too. One run of presses = one undo. |
| **speed** (under each target) | That move's speed: the default (`ARM_SPEED` / `SCOOP_SPEED`), `ARM_FAST` / `SCOOP_FAST` (the servo's own top speed, the old INSTANT), or a number. Every servo move is a smooth profile with the servo's accel (`ARM_ACCEL` / `SCOOP_ACCEL`). A new target takes over a move that's still running without a jerk. |
| **Slides / speed** | Target in mm, and a **named** speed (`SLIDE_FAST`, `SLIDE_CREEP`, `SLIDE_RETRACT_SPEED`; the retract speed brings `SLIDE_RETRACT_ACCEL`). Slide speeds and accels are fixed values you change in Angles & values, not per step. Two slide steps in a row, same direction, the first ending "when its moves finish": they **blend** with no stop (EXTEND → CREEP). |
| **Shake** | Vibrates the scoop ±N° at 10 Hz while the step runs. |
| **Next step** | After N ms · when **all** this step's moves have arrived (+ N ms) · when **one** axis has arrived (arm / scoop / slides, + N ms; its move may have started in an earlier step) · when the arm / scoop / slides are N% of the way. Example: SWEEP_UP on "arm arrived" ends the moment the arm reaches A_PEAK, so dragging that ◆ moves the whole rest of the run with it. Use the % options to start the next move early (CARRY waits for the arm at 90%). |

- **The table** has the same menus: each axis cell is the target with its speed under it. **type a number…** for anything else (servo speeds are capped at what the servo can do).
- **Add a step:** the **+ Add step** button above the table (also **+ Add step at the end** under the table, and **+ Add step after this** in the Step & move tab). Pick what it should do (move the arm, the scoop, both, the slides, shake, wait or empty); it goes after the selected step and opens in the Step & move tab.
<img src="tools/img/tuning_timeline.png" width="900" alt="Timeline with a diamond at the end of every move; the S_POUR diamond is being dragged and a label shows its old and new angle">

- **Change a move on the timeline, like keyframes in a video editor:** every move a step starts has a **◆** where it ends (blue arm, orange scoop, green slides; hollow = cut short by a later step before it got there). Hover one to see which step and name it is. **Drag it up or down** to change the position: if it's a name (`A_DOWN`), every step that uses that name changes, and the label says which. **Drag it sideways** (servos) to change when it arrives: the editor works out the speed that makes it arrive there (from where and how fast the servo really is when the step starts). Drag in any direction to do both at once. Slide ◆ only move up/down (their speeds are fixed). **Shift** = fine. **Click a ◆** to select it (ringed), then **↑ ↓** move the position (0.5° / 1 mm, Shift ×10, Alt ×0.2) and **← →** move when it arrives (10 ms steps); Esc deselects. The twin (and the real robot, with **Robot follows timeline** on) shows it while you drag. Esc cancels, Ctrl+Z undoes.
- **Change a step's time by dragging:** on the timeline, drag the edge between two blocks; it changes how long the left-hand step takes. A label shows old → new and which number it changed (Esc cancels):
  - *after N ms* → N;
  - *when the moves finish* → the extra wait (to make the moves themselves quicker, drag their ◆ sideways);
  - *arm / slides N% of the way* → the %.
  If it won't go further, the label says why (for example a smooth move can't finish faster than `ARM_SPEED` allows).
- **Reorder:** drag ⠿ at the left of a row. **Duplicate** / **Delete** are at the bottom of the tab.
- **⚠** next to a step's time means something to check:
  - a step that ends straight away because it waits on an axis that isn't moving;
  - **spill**: the scoop pours after it has picked up rocks but before the arm is back over the bucket (arm more than 25° forward of the bucket side). It's also marked on the strip under the timeline. It's a check on the twin's angles, so it is only as good as the calibration.
- Faded cells in the table are HOLD.

## 4. Change a pose with the sliders (no robot needed)

<img src="tools/img/tuning_jog_tab.png" width="460" alt="Pose sliders tab: ARM, SCOOP and SLIDES sliders, each with a number box, a load-a-name menu and a Save button, and a readout saying whether the scoop holds rocks, with the servo pulses">

The quickest way is the step's value boxes (§3: ↑ ↓). The **Pose sliders** tab is still there for free posing:

1. In the menu next to a slider, pick the name to change (say `S_POUR`). The slider loads its value and the twin goes there. (**Show** next to a name in **Angles & values** does the same.)
2. Drag the slider until the twin looks right. The line under the sliders says whether the scoop holds or pours, and the pulse widths the servos will get. The grey ghost is the name's current value (before / after).
3. **Save** on that row. Every step that uses the name now uses the new value.
4. **Copy pose from the timeline** copies the pose at the playhead into the sliders.

Need a new pose? **Angles & values** → **Add** in that group. Click its name to rename it (every step that uses it follows), then pick it in a step. **✕** deletes a name no step uses (names the robot's code needs, like `A_START`, can't be deleted).

## 5. Checks before you upload

- [ ] No red error line under the title, no ⚠ on any step (no "spill" marker on the strip).
- [ ] Scoop strip **green** from the end of SWEEP_DOWN until the arm is over the bucket, **red** at POUR.
- [ ] No servo speed flagged in yellow (faster than the servo can go).
- [ ] Nothing passes through anything in the twin: arm vs slides during EXTEND, scoop vs bucket at START and TRANSFER. Known issue: the CAD START pose overlaps the bucket a little until the servos are calibrated.
- [ ] Predicted run time is well under the limit (the robot forces DONE at 60 s).
- [ ] `SLIDE_FAST` / `SLIDE_RETRACT_SPEED` under about 80% of the free speed shown at the bottom of Angles & values.

**Save**, then upload `Robot_Controller_V2_1.ino` in the Arduino IDE.

## 6. Try it on the robot

1. Upload `Robot_Controller_V2_1.ino` once (no settings to change), close the Arduino Serial Monitor, battery on.
2. **Connect robot** (top right). Under the twin, **hold still and wait** is picked: connecting restarts the robot and the editor tells it to hold still within 0.15 s. The **Robot** panel's badge says **Ready**, and the button turns green.
3. The **Robot** panel under the twin always has:
   - **■ Stop**: stops everything (run, slides, shaking). Always works.
   - **↺ Reset to start**: servos to `A_START` / `S_START` (the IDLE pose), slides to 0 mm at 300 mm/s, timeline to 0 s.
   - **Limp**: all three servos and the slides go loose so you can move them by hand. The button stays red while limp; click it again to power the servos at the pose the twin shows.
   - **Robot follows timeline**: see below. **Slides follow too** off = arm and scoop only.
   - The robot's messages, and a box to type any console command (`Q` prints the pose, `?` lists them).
4. **Without uploading: Robot follows timeline.** Turn it on (under the twin, or next to the timeline's play bar) and the real robot copies the twin: scrub the timeline, press play, drag a ◆, drag a pose slider. Or select a step and press **▶ Play the edited sequence on the robot** (Step & move tab). Before playing, the robot first moves to the starting pose with smooth jog moves (no jump) and the play button says **Lining up…**. **⏮ Step start** then **▶ Play step** runs one step at a time this way. The difference: **▶ Play step** plays only the selected step (on the twin, and on the robot if it's following), stopping where the next step would start; **▶ Play from this step to the end, on the robot** turns following on and plays everything from the selected step to the end of the run. This runs what's **on screen**, so you can edit, play, edit again with no upload. It's the editor's timing, not the robot's: a stall or a slow servo won't hold the timeline up, and the slides follow the planned path rather than running their own profile.
5. **The uploaded sequence** (what the robot does at the competition): **Run all** / **Run from the selected step**. **Pause after each step** (the button fills while it's on) + **Next step** goes one step at a time. The **speed** menu (0.25× = 25 %) slows every move and wait on the robot too. While it runs, the twin shows the real robot.
6. When it's right: **Save**, upload, and check one full run with **Run all**. Competition: nothing to change. With no computer attached, the robot waits 0.15 s and runs.

Not working? The badge and the red box under it say why (for example an older program on the robot: upload the latest V2.1 once).

## 7. Keeping the code and the editor in step

`Sequence.h` is the only copy of the sequence. **Whichever was edited last wins** (AGENTS.md rule 7):

- Edited `Sequence.h` by hand or with an agent? Opening the folder in the editor reads it as it is. Ask the agent to refresh the editor's built-in snapshot too (that is only what the page shows before a folder is opened).
- Used the editor without the folder (a browser with no folder access)? **Download** or **Copy**, replace `Sequence.h` with it, and tell your agent.
- Keep the file's format if you edit by hand: `const float NAME = value;  // note` lines under the `// ---- … ----` headers, one `{ "STEP", … },` row per step. The editor shows a red error if it can't read something.
