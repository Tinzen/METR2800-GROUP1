# How the V2.1 state machine works

V2.1 is still a finite state machine. It's written as a **table plus one generic runner** instead of a `switch` with one `case` per state. This file maps the two views onto each other, so you can read, predict and change every transition.

Files: `Sequence.h` (the table), `Robot_Controller_V2_1.ino` (the runner: `enterStep()`, `stepConditionMet()`, `updateRunner()`), `RobotConfig.h` (hardware numbers).

---

## 1. FSM concepts → where they live

| Classic FSM | V2.1 |
|---|---|
| State | One **row** of `SEQUENCE[]` (its name is the state name in the logs) |
| Current state | `stepIndex` (row number). Logged as `S,<ms>,<NAME>,<code>` on every change |
| Entry action | `enterStep(i)`: reads the row and **starts** the arm, scoop and slide moves and the vibration |
| During action | Every `loop()`: servo profiles, slide PID (every 10 ms), safety monitors, telemetry |
| Transition guard | The **NEXT WHEN** column, checked by `stepConditionMet()` every loop, **or** the row's **OR AFTER** time limit |
| Next state | Always the **next row**. The only jumps are the fault transitions (section 4) |
| Exit action | None. A move still running when its row ends **keeps going** (that's how steps overlap) |
| Top-level states | `runState`: `R_IDLE` → `R_RUNNING` ⇄ `R_PAUSED` (step mode) → `R_DONE`. Plus `consoleMode` (bench): set by `b` within 150 ms of power-up (the editor's **Hold still**) or by `RUN_CONSOLE`; the robot then waits for commands instead of running |

So the step machine is linear, `row 0 → 1 → … → last → DONE`, with a few fault exits. Everything a `case` would do is described by the row's columns.

---

## 2. One loop() pass, in order

```
handleSerial()          console commands (x always works)
slide.update()          slide profile + PID + feedforward, every 10 ms
updateSlideMonitors()   stall / settled / wiring-direction checks
jog timeout, 60 s watchdog
updateRunner()          the FSM: timed servo moves, slide faults, guard, transition
arm.update(), scoop.update()   servo profiles / instant estimates, vibration, 50 Hz writes
logTelemetry()          D line every 20 ms
```

`updateRunner()` is the whole FSM:

```cpp
if (runState != R_RUNNING) return;
if (a slide move is still on its way) {    // even if an earlier row started it
  if (stalled for 200 ms, after the first 300 ms)  -> slideFault("STALL")
  if (step time > profile time + 1.5 s, not arrived) -> slideFault("TIMEOUT")
}
if (!stepConditionMet()) {                // guard (NEXT WHEN)
  if (no OR AFTER limit, or step time < limit) return;
  logEvent("STEP_LIMIT", stepIndex);      // stuck: move on anyway
}
if (stepIndex >= lastStep) finishRun();   // -> DONE
else if (stepMode) runState = R_PAUSED;   // wait for 'n'
else enterStep(stepIndex + 1);            // -> next state
```

---

## 3. What each column does (entry actions)

`{ "NAME", ARM, deg/s, SCOOP, deg/s, SLIDE, SLIDE mm/s, VIBRATE, NEXT WHEN, OR AFTER, A.ACC%, A.DEC%, S.ACC%, S.DEC% }` (the last five may be left out: OR AFTER = 0 = no limit, the % = 100)

**Every move is a trapezoid motion profile** (speed up at the axis's `_ACCEL`, cruise, slow down at its `_DECEL`). There is no "instant" or "timed" mode any more (16 Oct): a fast move is just a profile with a high speed. With a different accel and decel the trapezoid is lopsided; a move too short to reach its speed peaks where the two ramps meet.

**ARM / SCOOP** (degrees, a named angle, or `HOLD`) and the **deg/s** after each
- A value: this row **takes over** that axis with a new profile to that angle, at that speed (0 = `ARM_SPEED` / `SCOOP_SPEED`) with the servo's accel and decel (`ARM_ACCEL` / `ARM_DECEL`, `SCOOP_ACCEL` / `SCOOP_DECEL`), each scaled by the row's ACC% / DEC% (100 = as is).
- **Interrupting is smooth.** If the axis is still moving from an earlier row, the new profile starts at the speed it's going: same direction = it carries on into the new move (if it's going faster than the new speed, it slows down to it at the decel); opposite direction = it brakes to a stop at the decel first, then goes. No jerk either way (`MotionProfile::moveFrom`).
- `HOLD`: the row doesn't touch that axis. A move still running from an earlier row **keeps running**. That's how servos move asynchronously: start a move in one row, change state on a timer (`WAIT_MS`) while it finishes.
- `ARM_FAST` (358 °/s) / `SCOOP_FAST` (430 °/s) are the servos' own top speeds: the old `INSTANT`. Faster than that and the servo can't keep up, so "arrived" (profile finished) comes before the servo really gets there; the editor flags it.

**SLIDE** (mm from fully retracted, or `HOLD`) and **SLIDE mm/s** (a named speed, not a number)
- A value: a profiled slide move at that speed with `SLIDE_ACCEL` / `SLIDE_DECEL`. `SLIDE_RETRACT_SPEED` comes with its own `SLIDE_RETRACT_ACCEL` / `SLIDE_RETRACT_DECEL` (the RETRACT row). If the slide is still moving the same way (a row that ended early with `- ms`), the new move starts at that speed. 0 = `SLIDE_FAST`. The controller converts mm to counts with `SLIDE_MM_PER_COUNT`.
- **Blend:** if this row's guard is `WAIT_ARRIVED` (or `WAIT_SLIDE_ARRIVED`) and the next row moves the slide **further in the same direction**, this row doesn't stop. It ends at the slower of the two speeds and hands that speed to the next row. That's EXTEND → CREEP: fast, then creep, with no stop between.

**VIBRATE** (± degrees on the scoop): `> 0` starts the shake (10 Hz). A row with `0` **stops** any shake still running. (The editor shows it in the Step tab only.)

**NEXT WHEN** (the guard)

| Guard | Fires when |
|---|---|
| `WAIT_MS(t)` | t ms after the row started, whatever the moves are doing (the usual choice for open-loop servos) |
| `WAIT_ARRIVED(t)` | **every axis has stopped**: arm and scoop, and the slides once any row has moved them this run, including moves started in earlier rows (16 Oct; before, only the axes this row moved). Then t ms more. Servo "arrived" = its profile finished. Slide "arrived" = profile finished and within `SLIDE_ARRIVE_TOL_MM`; or stopped (settled) within `SLIDE_SETTLED_TOL_MM` (logs `SETTLED_SHORT`); or handing off a blend |
| `WAIT_ARM_ARRIVED(t)` / `WAIT_SCOOP_ARRIVED(t)` / `WAIT_SLIDE_ARRIVED(t)` | just that one axis has arrived, then t ms. Its move may have started in an earlier row (e.g. SWEEP_UP can end the moment the arm reaches A_PEAK while the scoop keeps turning) |
| any arrived guard with **t < 0** | moves on \|t\| ms **before** the planned arrival (the profiles' `getTimeLeft()`, i.e. the plan, not a measurement), so the next row's move overlaps and blends in |
| `WAIT_ARM(p)` / `WAIT_SCOOP(p)` | that servo is p % through its current move (the move may have started in an earlier row) |
| `WAIT_SLIDE(p)` | the slide profile is p % through |

**OR AFTER** (ms, `uint16_t`, 0 = none): a time limit on the row. If NEXT WHEN still isn't true after that long (a slide stuck short of its target, a stalled move), the row moves on anyway and logs `E,<ms>,STEP_LIMIT,<row>`. It doesn't stop any move, it only changes state.

**Speed scaling:** `k` / `SPEED_PCT_AT_BOOT` (the editor's playback speed sends `k`) multiplies every speed by k and every acceleration and deceleration by k², and divides every wait and limit by k, so `k 25` replays the whole run at quarter speed, with the same shape.

---

## 4. Transitions that skip rows (faults)

| Event | Where | Goes to |
|---|---|---|
| Slide `STALL` (PWM ≥ 90, under 50 counts/s, for 200 ms) | `updateRunner` → `slideFault()` | the `RETRACT` row, **once** (`FALLBACK` event). A second fault, or no RETRACT after the current row → DONE |
| Slide `TIMEOUT` (profile time + 1.5 s, not arrived) | same | same |
| OR AFTER limit (`STEP_LIMIT`) | `updateRunner` | the **next** row, like a normal transition (not a fault) |
| `DIR_MISMATCH` (motor driving one way, encoder running the other) | `updateSlideMonitors` | DONE (motor coasts) |
| `x` on serial | `processCommand` | DONE |
| `b` on serial | `processCommand` | DONE, then bench mode (waits for commands) |
| 60 s watchdog | `loop` | DONE |

DONE = `finishRun()`: slide coasts, servos freeze where they are, end LED on, `R,<ms>,<run time>` logged, config printed.

**This is exactly what happened in every 15 Oct run:** `X_TIP` is past the end stop, so CREEP pushed into the stop → `STALL` → `FALLBACK` to RETRACT. TIP_HOLD never ran.

---

## 5. The current sequence as a classic switch FSM

This is what the table + runner does, written the way you're used to. It's equivalent logic, not code to paste.

```cpp
switch (state) {
  case IDLE:        // entry: arm -> A_START at ARM_FAST, scoop -> S_START at SCOOP_FAST (already there at power-up)
    if (arm_done && scoop_done) state = SCOOP_DOWN; break;
  case SCOOP_DOWN:  // entry: scoop -> S_DOWN at SCOOP_FAST
    if (t_in_state >= 300) state = ARM_DOWN; break;
  case ARM_DOWN:    // entry: arm -> A_DOWN at ARM_FAST                (scoop may still be moving)
    if (t_in_state >= 500) state = SWEEP_UP; break;
  case SWEEP_UP:    // entry: arm -> A_PEAK at 62.1 deg/s (takes over ARM_DOWN's move smoothly),
                    //        scoop -> S_SCOOP at 156.1 deg/s (it carries on through SWEEP_DOWN)
    if (arm_done) state = SWEEP_DOWN; break;
  case SWEEP_DOWN:  // entry: arm -> A_SCOOP at 75 deg/s               (scoop: HOLD, still turning)
    if (arm_done && scoop_done) state = WAIT; break;       // WAIT_ARRIVED: the scoop's SWEEP_UP move too
  case WAIT:        // entry: nothing
    if (t_in_state >= 120) state = PICKUP; break;
  case PICKUP:      // entry: scoop -> S_PICKUP at SCOOP_FAST
    if (t_in_state >= 260) state = LIFT; break;
  case LIFT:        // entry: arm -> A_TRANSFER at 222.1 deg/s
    if (t_in_state >= 100) state = CARRY; break;          // arm still lifting
  case CARRY:       // entry: scoop -> S_CARRY at 210.6 deg/s     (arm: LIFT's move keeps going)
    if (arm_done) state = POUR; break;                     // LIFT's arm move finished
  case POUR:        // entry: scoop -> S_POUR at SCOOP_SPEED
    if (arm_done && scoop_done) state = VIBRATE; break;
  case VIBRATE:     // entry: scoop.startVibration(7.5 deg, 50 ms)
    if (t_in_state >= 590) state = EXTEND; break;
  case EXTEND:      // entry: stop vibration; arm -> A_MOVE, scoop -> S_DOWN; slide -> X_CREEP at SLIDE_FAST,
                    //        ending at SLIDE_CREEP speed (blend)
    if (arm_done && scoop_done && slide_profile_done) state = CREEP; break;
  case CREEP:       // entry: slide -> X_TIP at SLIDE_CREEP, starting at SLIDE_CREEP speed
    if (arm_done && scoop_done && slide_arrived) state = TIP_HOLD; break;   // X_TIP past the stop: STALL -> RETRACT
  case TIP_HOLD:    // entry: nothing (everything holds)
    if (t_in_state >= 1500) state = RETRACT; break;
  case RETRACT:     // entry: slide -> 200 mm at SLIDE_RETRACT_SPEED / _ACCEL / _DECEL
    if (arm_done && scoop_done && slide_arrived) state = DONE; break;
}
// every state: if OR AFTER is set and t_in_state >= it -> next state (STEP_LIMIT)
// every state: if slide stalls/times out -> RETRACT (once), else DONE
```

Things the table makes easy to miss:
- **CARRY's guard watches the arm move LIFT started.** CARRY doesn't move the arm (HOLD), so `WAIT_ARM_ARRIVED(0)` waits for LIFT's still-running move.
- **`WAIT_ARRIVED` waits for everything**, even moves an earlier row started (SWEEP_DOWN waits for the scoop's SWEEP_UP move). To wait for one axis only, use its `WAIT_x_ARRIVED`.
- **ARM_DOWN starts 500 ms after SCOOP_DOWN whether or not the scoop has finished.** `WAIT_MS` never waits for motion.
- **Servo "done" is the profile finishing**, not a measurement: hobby servos can't report back.

---

## 6. Getting more control

**1. Most changes are table edits.** Reorder rows, change a guard, split a step in two, add a row. The editor (`tools/sequence_editor.html`) previews the timing on the twin before you upload.

**2. Bench controls** (top of the `.ino`, or live over serial in bench mode: connect with the editor's **Hold still**, or send `b`):

| Setting / command | Does |
|---|---|
| `FIRST_STEP` / `LAST_STEP`, `r <n>` | run only part of the sequence |
| `STEP_MODE_AT_BOOT`, `t` then `n` | pause after every state; `n` = next |
| `SPEED_PCT_AT_BOOT`, `k 25` | whole run in slow motion, same shape |
| `A <deg>`, `S <deg>`, `o <mm>`, `Q` | jog any axis / print the pose (between runs) |
| `P <arm> <scoop> <mm> <mm/s> <vib>` | what the editor's Drives: **Robot · live** streams: try an edited sequence without uploading (it bypasses this state machine: the editor's `simulate()` is the runner) |

**3. Custom code for one state.** For things the columns can't express (a sensor, a branch, a loop), there are three places to hook in:

- **Custom entry action:** in `enterStep()`, after the row is loaded:
  ```cpp
  if (strcmp(cur.name, "POUR") == 0) { /* your code */ }
  ```
- **Custom guard:** add a value to `WaitType` in `SequenceTypes.h` (e.g. `W_CUSTOM`) and a `case` in `stepConditionMet()`.
- **Branch or loop:** in `updateRunner()`, replace `enterStep(stepIndex + 1)` with your own choice, e.g. `enterStep(findStep("SWEEP_UP"))` to repeat the intake.

The editor's `simulate()` copies the firmware's rules. If you add custom logic, the twin won't know about it unless `simulate()` gets the same change (AGENTS.md rule 7). Flash is at about 28.6 KB of 30 KB here (about 1 KB more in the IDE), so keep hooks small and strings in `F()`.

---

## 7. Watching it run

- Every state change prints `S,<ms>,<NAME>,<code>`, and every event prints `E,<ms>,<CODE>,<value>` (STALL, FALLBACK, SETTLED_SHORT, STEP_LIMIT, VIB_ON…).
- `tools/run_viewer.html` shows the run as a Gantt of states, plots and the 3D replay.
- The editor's live link shows the robot's current state and pose on the twin as it runs.
