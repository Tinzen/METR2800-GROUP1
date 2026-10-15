# Slide Test

Moves the outtake slides to a position you type, in **mm**, with a smooth motion profile: it speeds up, cruises, then slows down. It can also log every move as a graph. Nothing moves until you type a command.

- Positions are in mm from wherever you last typed `z`. 1 encoder count = 0.5317 mm (36.15 mm spool, 1.8:1 step-up).
- Pins: motor MA = D8, MB = D9, PWM = D11; encoder D2 / D3.
- **Battery must be ON.** With USB only, the encoder count drifts and moves abort.

## Commands

### Moving

| Type | Does | Example |
|---|---|---|
| `z` | Zero: "this spot is 0 mm". Do it fully retracted. | `z` |
| `m <mm>` | Move smoothly **to** a position | `m 100` → goes to 100 mm |
| `m <mm> <vel>` | Same, at a one-off top speed (mm/s) | `m 500 800` |
| `m <mm> <vel> <acc>` | Same, with a one-off speed and acceleration | `m 500 800 3000` |
| `mr <mm>` | Move smoothly **by** a distance from where it is now (also takes `<vel> <acc>`) | `mr 100` → 100 mm further out, `mr -50` → 50 mm back |
| `r` | Go back to 0 mm | `r` |
| `e` | Go to the `ext` position | `e` |
| `j <pwm> <ms>` | Raw jog, no smoothing (pwm -255 to 255) | `j 100 300` |
| `x` | **STOP.** Always works. | `x` |
| `c` | Motor off (slide goes loose) | `c` |
| `h` | Hold the current position | `h` |

`m` vs `mr`: `m 100` always ends at the 100 mm mark. From 0 that's 100 mm of travel; from 300 it moves back 200 mm. Use `mr 100` to move 100 mm from wherever it is now.

### Speed and acceleration

| Type | Does | Example |
|---|---|---|
| `vel <mm/s>` | Top speed for all moves | `vel 600` |
| `acc <mm/s²>` | How hard it speeds up and slows down | `acc 2000` |
| `ext <mm>` | Position used by `e` and `t` | `ext 450` |
| `s` | Show position, target, ext, vel and acc | `s` |

These live changes are **lost on restart**. To change the start-up values, edit near the top of `Slide_Test.ino`:

```cpp
const float DEFAULT_VEL_MM_S  = 300.0f;    // mm/s
const float DEFAULT_ACC_MM_S2 = 1000.0f;   // mm/s^2
```

Rough guide: the motor's top speed is about 1480 mm/s with no load. Above about 1100 mm/s the motor runs out of power and the slide lags behind the profile (the summary line shows a large `peak lag`). Raise `vel` in steps (400, 600, 800…) and watch `peak lag` and `final err`.

### Output and tuning

| Type | Does |
|---|---|
| `d` | Switch output: raw data (for Python) ↔ readable mm (for Serial Monitor) |
| `kp ki kd ks kv ka kg <value>` | Change a gain live, e.g. `kp 0.6` |
| `t <n> [dwell_ms]` | Cycle 0 → ext → 0, n times |
| `f` | Feedforward test (7 moves, for tuning) |
| `p` | Print settings to paste into `Robot_Controller_V2_1.ino` (in counts) |
| `?` | Help |

**There's no travel limit.** Moves go wherever you ask; only targets below 0 are set to 0. If you ask for more than the slide can travel, it will drive into the end stop. The stall cut-out (motor pushing but not moving for 0.2 s) then stops it and reports `STALL`. Know your travel before typing big numbers.

After every move it prints one line:

```
# move 0.0 -> 100.0 mm @300 mm/s: planned 0.63 s, took 0.64 s (ARRIVED), now at 101.1 mm, peak lag 12.1 mm, overshoot 1.1 mm, final err -1.1 mm
```

`ARRIVED` means it reached the target. After a move it holds the position gently for 10 s, then lets go.

## Finding `kg` and `ks` (about 5 minutes)

`kg` is a constant push that cancels whatever pulls the slide back in (gravity on a tilt, string or spring tension). `ks` is the extra push needed to overcome friction. You find both with low-power jogs in `Slide_Test` and type them in **live**; no code change is needed.

1. `z` (retracted), then `m 200` to put the slide mid-travel.
2. Jog with small powers for 2 seconds each and read the `# jog done, pos …` line after each one:
   ```
   j 5 2000
   j 10 2000
   j 15 2000
   j 20 2000     ...keep going up by 5
   ```
   - Note **A** = the highest power where the slide still **slides back in** (position goes down).
   - Note **B** = the lowest power where it starts **moving out** (position goes up).
   - In between, it just sits still: friction is holding it.
3. Set:
   - `kg` = (A + B) / 2, e.g. A = 10, B = 30 → `kg 20`
   - `ks` = (B - A) / 2, e.g. → `ks 10`
4. Check: `m 200` and watch that it **holds steady at the target** (HOLD) without creeping back.

If the slide never slides back (A doesn't exist), there's nothing pulling it in: leave `kg 0` and set `ks` = B.

Type gains like any command: `kg 20` + Enter. They apply immediately and are **lost on restart**. Use `p` (here) or `g` (in `Slide_PID_Tune`) to print lines you can paste into the code to keep them.

---

## Option A: Serial Monitor (quick)

1. Upload `Slide_Test.ino`.
2. Open the Serial Monitor at **115200** baud.
3. Battery ON.
4. Type `d` + Enter for readable output: `pos 84.0 mm   target 100.0 mm   pwm 24   HOLD`
5. `z`, then `m 100`, `m 300`, `r`…

## Option B: Python logger (saves graphs)

One-time setup: Python and pyserial, step by step in [`Robot_Controller_V2_1/LOGGING.md`](../../Robot_Controller_V2_1/LOGGING.md) (Part 1). Its Troubleshooting table covers "Access is denied", missing ports and the other usual problems.

1. Upload from the Arduino IDE, then **close the Serial Monitor**.
2. In File Explorer open `METR2800-GROUP1`, type `cmd` in the address bar, Enter. Keep this window open.
3. Battery ON.
4. Run:
   ```
   python tools/capture_log.py --port COM5 --keep-open --label slidetest
   ```
   The Nano restarts, which is normal. Leave the output on raw data (don't type `d`).
5. Type commands + Enter (`z`, `m 100`, `r`…). The `# move` summaries are in mm. The live `slide=` numbers are raw counts (divide by 1.88 to get mm).
6. **Ctrl+C** when done. Open the new `.html` file in `logs/` to see the graphs.

To change code: Ctrl+C, upload, then **Up arrow + Enter** in the black window.

---

## Problems

| You see | Means | Try |
|---|---|---|
| Count changes on its own | Battery off | Turn the battery on |
| `SETTLED_SHORT` (stops a few mm early) | Not enough push near the end | `kp 0.8`, or `ks 20` |
| Overshoots past target | Too much push or too fast | Lower `kp`, or lower `vel` / `acc` |
| Large `peak lag` | Asking for more speed than the motor has | Lower `vel` / `acc` |
| `DIR_MISMATCH` | Motor and encoder disagree on direction | Flip `OUTTAKE_MOTOR_DIRECTION` |
| `NO_MOTION` | Motor pushing but encoder not counting | Battery off, or motor / encoder unplugged |
| `STALL` | Pushing but not moving | Hit the end of travel or jammed |
| `busy: x to stop first` | A move is still running | Wait, or type `x` |
