# Tuning the sequence digitally

Get the whole sequence right in the editor first. The robot is then only for confirming it. The tool is `tools/sequence_editor.html`: Chrome or Edge, with `three.min.js` and `twin_model.js` next to it. It reads and writes `Robot_Controller_V2_1/Sequence.h`, the same file the robot runs.

<img src="tools/img/tuning_overview.png" width="900" alt="Sequence editor: 3D twin with a play bar at the top left, the Step tab at the top right, the timeline below, and the step table at the bottom">

## 0. How far you can trust the twin

| The twin is right about | The twin can't tell you |
|---|---|
| Order and overlap of steps, timed and profiled moves, slide profiles and blending, predicted run time | How fast a loaded servo really moves (`INSTANT` jumps assume about 430 °/s) |
| Whether the scoop holds, is level or pours (the coloured strip) | Slide lag and stalls: the sim follows the profile perfectly, the real slides lag 0.1 to 0.2 s |
| Collisions you can see, once the angles are calibrated | Rocks |

- **Servo angles are only real after calibration.** Right now they come from Tinzen's note that the V2 SCOOP pose lies flat on the floor. Do `calibration/README.md` §4 when you can.
- **Slide tilt** (box above the twin): 14.9° is a placeholder. Measure the real angle and type it in. It only changes the picture.
- **Slide positions** `X_CREEP` / `X_TIP` are safe placeholders (450 / 530 mm). Find the real ones with `calibration/Slide_Test`. The CAD says about 1650 / 1740.

## 1. Open the robot folder

Click **Open robot folder** and pick the repo (or `Robot_Controller_V2_1`). The chip next to the title shows the open file. **Save** (Ctrl+S) writes `Sequence.h`, and `RobotConfig.h` too if you changed the calibration. Chrome remembers the folder: next time, click **Reopen**.

## 2. Watch the sequence

- **Play**: the button on the twin, or Space. **‹ ›** (or ← →) jump one step. 0.25× is good for fast steps.
- **Timeline**: step blocks on top (click one to select it). Under them is the scoop strip: **green** holds rocks, **yellow** level, **red** pours. Then the traces: arm (blue), scoop (orange), slides (green, right axis).
- The **grey ghost** in the twin is where the arm and scoop are heading (the target). The solid model is where they are now. While you edit or jog an angle, the ghost shows the old value instead (before / after).

3D view, same controls as Fusion 360 / Inventor:

| Do | Mouse | Trackpad |
|---|---|---|
| Zoom | scroll, zooms at the cursor | two-finger scroll or pinch |
| Pan | middle-drag | Shift + drag |
| Orbit | Shift + middle-drag, about the point under the cursor | drag |
| Fit the robot | double-click (middle or left) | double-tap |

Scroll the wrong way round for you? Tick **Reverse zoom** under the twin. The **View** menu jumps to Robot / Side / Field.

## 3. Change what a step does: the Step tab

Click a step's **number** or **description** in the table (or its block on the timeline). The **Step** tab opens. It says in plain words what the step does, when it starts and how long it takes.

<img src="tools/img/tuning_step_tab.png" width="460" alt="Step tab for SWEEP: a plain-English summary, arm and scoop menus, move type radio buttons, arm via, slides, shake and next-step options">

| Control | What it means |
|---|---|
| **Arm / Scoop** | A named angle, a number, or **carry on (HOLD)**. HOLD doesn't stop the axis: a move started in an earlier step keeps going. That is how steps overlap. |
| **Moves** | **Jump**: send the angle at once, like V2. **Smooth**: accelerate and brake with `ARM_SPEED` / `SCOOP_SPEED`. **Take exactly N ms**: arm and scoop finish together. |
| **Arm via** | Only for timed moves: the arm passes through this angle halfway (the V2 sweep lifts to `A_PEAK`). |
| **Slides / Speed** | Target in mm and speed in mm/s. Two slide steps in a row, same direction, the first one ending "when its moves finish": they **blend** with no stop (EXTEND → CREEP). |
| **Shake** | Vibrates the scoop ±N° at 10 Hz while the step runs. |
| **Next step** | After N ms · when this step's moves finish (+ N ms) · when the arm is N% of the way · when the slides are N% of the way. Use the % options to start the next move early (CARRY waits for the arm at 90%). |

- **Add step…** (above the table) inserts a ready-made step after the selected one: arm move, scoop move, both, slides, shake, wait.
- **Change a step's time by dragging:** on the timeline, drag the edge between two blocks; it changes how long the left-hand step takes. A label shows old → new and which number it changed (Esc cancels):
  - *after N ms* → N;
  - *when the moves finish* → the extra wait, or the move time if you drag shorter than a timed move;
  - *arm / slides N% of the way* → the %.
  If it won't go further, the label says why (for example a smooth move can't finish faster than `ARM_SPEED` allows).
- **Reorder:** drag ⠿ at the left of a row. **Duplicate** / **Delete** are at the bottom of the tab.
- **⚠** next to a step's time means something to check:
  - a step that ends straight away because it waits on an axis that isn't moving;
  - **spill**: the scoop pours after it has picked up rocks but before the arm is back over the bucket (arm more than 25° forward of the bucket side). It's also marked on the strip under the timeline. It's a check on the twin's angles, so it is only as good as the calibration.
- Faded cells in the table are HOLD. You can still type straight into the table if you prefer.

## 4. Change a pose: Jog, then Capture (no robot needed)

<img src="tools/img/tuning_jog_tab.png" width="460" alt="Jog tab: arm, scoop and slide sliders, a Pose at playhead button, a Capture into menu set to S_SCOOP, and a readout saying the scoop holds rocks with the servo pulses">

1. **Angles & values** → **Show** next to the name (say `S_POUR`). The twin goes to that pose, the **Jog** sliders copy it, and **Capture into** selects the name.
2. **Jog & live link** tab → drag **ARM / SCOOP / SLIDES** until the twin looks right. The line under the sliders says whether the scoop holds or pours, and the pulse widths the servos will get.
3. **Capture**. Every step that uses that name now uses the new value. Press play to check it in motion. (While jogging, the grey ghost is the pose with the name's current value, so you see before and after.)
4. **Pose at playhead** copies the pose at the current time into the sliders: a quick way to start from "where it is at this moment".

Need a new pose rather than changing one? **Angles & values** → **Add** in that group, rename it, then use it in a step.

## 5. Checks before you upload

- [ ] No red error line under the title, no ⚠ on any step (no "spill" marker on the strip).
- [ ] Scoop strip **green** from the end of SWEEP until the arm is over the bucket, **red** at POUR.
- [ ] Nothing passes through anything in the twin: arm vs slides during EXTEND, scoop vs bucket at START and TRANSFER. Known issue: the CAD START pose overlaps the bucket a little until the servos are calibrated.
- [ ] Predicted run time is well under the limit (the robot forces DONE at 60 s).
- [ ] `SLIDE_FAST` / `SLIDE_MAX` under about 80% of the free speed shown at the bottom of Angles & values.

**Save**, then upload `Robot_Controller_V2_1.ino` in the Arduino IDE.

## 6. Confirm on the robot

1. In `Robot_Controller_V2_1.ino` set `RUN_MODE = RUN_CONSOLE`, upload, close the Arduino serial monitor, battery on.
2. Editor → **Jog & live link** → **Connect robot**. The twin now mirrors the robot.
3. **Step mode (t)**, then **Next step (n)** to go through one step at a time. **Speed 25%** slows smooth, timed and slide moves and the waits (Jump moves still go at servo speed). **Run all (g)** for the real thing.
4. Something off by eye? Use the **same sliders**: the real arm and scoop follow while you drag. **Send slides** moves the slides to the slider value at 300 mm/s. Then **Capture**, **Save**, re-upload.
5. Competition: set `RUN_MODE = RUN_FULL` again. Opening the serial port resets the Nano, and in RUN_FULL that starts a run.

## 7. Keeping the code and the editor in step

`Sequence.h` is the only copy of the sequence. **Whichever was edited last wins** (AGENTS.md rule 7):

- Edited `Sequence.h` by hand or with an agent? Opening the folder in the editor reads it as it is. Ask the agent to refresh the editor's built-in snapshot too (that is only what the page shows before a folder is opened).
- Used the editor without the folder (a browser with no folder access)? **Download** or **Copy**, replace `Sequence.h` with it, and tell your agent.
- Keep the file's format if you edit by hand: `const float NAME = value;  // note` lines under the `// ---- … ----` headers, one `{ "STEP", … },` row per step. The editor shows a red error if it can't read something.
