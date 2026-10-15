# Slide PID Tune

A separate sketch for tuning the outtake slide controller. You run the same test again and again, change one gain at a time, and compare the one-line scores. It uses the same pins, units and gains as `Slide_Test`. Battery ON.

## The two tests

| Type | Test | Use it for |
|---|---|---|
| `b 50 400 3` | **Bounce:** smooth moves 50 → 400 → 50 mm, 3 round trips, then an average score | Tuning the controller the way the robot actually uses it (profile + PID + feedforward) |
| `step 200` | **Step:** target jumps instantly to 200 mm, pure PID with no profile, PWM capped at 150 | Seeing raw P/D behaviour: how fast it reacts and whether it overshoots or oscillates |

Each move prints:

```
# move 47 -> 400 mm: peak lag 12.2 mm, overshoot 1.5 mm, settle 0.00 s, final err 1.7 mm
```

and a bounce test ends with:

```
# == BOUNCE RESULT (6 moves): avg lag 14.8 mm | avg overshoot 9.2 mm (worst 17.0) | avg settle 0.11 s | avg final err 2.2 mm ==
```

These lines appear **live in the black window (or Serial Monitor) after every move**, so you don't need the graph page to read peak lag. The graph page (on Ctrl+C) is for looking closer.

| Number | Means | Good |
|---|---|---|
| **peak lag** | Biggest gap between where the profile wants the slide and where it is, during the move | Small (a few mm). Large = too slow to follow: raise `kv`, or lower `vel` / `acc`. |
| **overshoot** | How far it went past the target | Close to 0 |
| **settle** | Time after the profile ends until it's within ±3 mm and stays there | Close to 0 s. `NEVER` = it never got within 3 mm. |
| **final err** | Distance from the target when the move finished | Under 3 mm |
| **rise** (step only) | Time to go from 10% to 90% of the way | Shorter = more responsive |

## Other commands

| Type | Does |
|---|---|
| `z` | Zero here (fully retracted) |
| `m 100` / `r` | Move to 100 mm / back to 0 |
| `kp ki kd ks kv ka kg <value>` | Change a gain, e.g. `kp 0.6` |
| `vel 600` / `acc 3000` | Profile speed (mm/s) / acceleration (mm/s²) |
| `g` | Show the gains and print lines to paste into `Robot_Controller_V2_1.ino` |
| `d` | Switch output: raw data (Python) ↔ readable mm (Serial Monitor) |
| `x` | **STOP** |

Gain changes apply immediately (even mid-test) and are **lost on restart**. Use `g` to copy them out.

## What each gain does

The motor command each moment is **PID + feedforward**. All gains are per encoder count (1 count = 0.53 mm), the same as V2_1.

| Gain | Kind | What it does | Too low | Too high |
|---|---|---|---|---|
| `kp` | PID | Push in proportion to the error | Lags, stops short | Overshoots, buzzes, oscillates |
| `kd` | PID | Brakes when closing in fast | More overshoot | Jittery, noisy motor |
| `ki` | PID | Slowly builds push if it's stuck short | Small steady error | Slow overshoot or wandering. Usually leave at 0 and use `ks`. |
| `ks` | FF | Fixed push to beat friction | Stops a few mm short | Twitches / overshoots small moves |
| `kv` | FF | Push in proportion to the planned speed | Lags behind the whole move | Runs ahead of the profile |
| `ka` | FF | Extra push while accelerating | Lags at the start and end of moves | Lurches |
| `kg` | FF | Constant push (gravity, if the slides are tilted) | Sags back when holding | Creeps forward |

## Tuning order (about 15 minutes)

0. **Find `kg` and `ks` first** with the jog method in `Slide_Test/README.md` (this sketch has no jog command). Type them here, e.g. `kg 20`, `ks 10`.
1. **Setup:** retract fully, `z`, then `vel 600`, `acc 3000` (or your target speeds).
2. **Baseline:** `b 50 400 3`. Write down the result line.
3. **`kv` first** (gets the slide moving at the right speed). If `peak lag` is large, raise `kv` in steps of 0.01 until lag stops improving or it starts running ahead.
4. **`ks`** (stopping short). If `final err` is consistently a few mm, raise `ks` in steps of 5.
5. **`kp`** (holding on). Raise it in steps of 0.2 until `overshoot` starts growing or the motor buzzes, then back off about 30%.
6. **`kd`** (only if it overshoots). Try 0.005–0.02. Stop if the motor gets noisy.
7. **Check with `step 200`.** It should rise quickly and settle with little overshoot.
8. **`g`**, then paste the lines into `Robot_Controller_V2_1.ino`.

Change **one** gain at a time and rerun the same bounce test. Otherwise you can't tell which change helped.

## Logging

This sketch outputs the same data format as `Slide_Test` and `V2_1`, so the Python logger works. First time? Set up Python and pyserial with [`Robot_Controller_V2_1/LOGGING.md`](../../Robot_Controller_V2_1/LOGGING.md).

```
python tools/capture_log.py --port COM5 --keep-open --label pidtune
```

Open the `.html` file afterwards. Position vs. setpoint is the clearest way to see lag and overshoot.

## Notes

- The arm parks itself out of the slide's path at start-up. Set `PARK_ARM = false` if the servos are unplugged.
- There's no travel limit. Pick test positions inside the slide's real travel.
- Safety: if the motor pushes (PWM ≥ 90) without moving for 0.3 s, it stops with `STALL`, or `NO_MOTION` if it never moved at all.
