# Logging a run with Python (first-time guide)

`tools/capture_log.py` records everything the robot prints over USB while it runs. Each run gives you two files in `logs/`:

| File | What it is | Open it with |
|---|---|---|
| `run_20261015_143012_mylabel.csv` | Raw telemetry: arm, scoop, slide position, motor PWM, every step and warning, 50 times a second | Excel, or drop it on `tools/run_viewer.html` |
| `run_20261015_143012_mylabel.html` | The run viewer with this run built in: graphs, step timeline, 3D twin replay | Double-click (Chrome or Edge) |

The same script works for `Robot_Controller_V2_1`, `calibration/Slide_Test` and `calibration/Slide_PID_Tune`.

You only do **Part 1** once per computer.

---

## Part 1: one-time setup (about 10 minutes)

### 1. Install Python 3

**Windows**

1. Go to **python.org → Downloads** and run the installer (any Python 3.9 or newer).
2. On the first screen, tick **"Add python.exe to PATH"**, then **Install Now**.
3. Check it worked: open **PowerShell** (Start menu → type `powershell`) and run:
   ```
   python --version
   ```
   You should see `Python 3.x.x`. If you get "not recognized", try `py --version`; if that works, type `py` instead of `python` everywhere below. If typing `python` opens the Microsoft Store, Python isn't installed yet: use the python.org installer.

**macOS**

1. Install from **python.org → Downloads** (or `brew install python` if you use Homebrew).
2. Open **Terminal** and check: `python3 --version`. On a Mac, type `python3` wherever this guide says `python`.

### 2. Install pyserial

pyserial is the library that lets Python talk to the USB serial port. In the same window:

```
python -m pip install pyserial
```

(Mac: `python3 -m pip install pyserial`.)

- The package is called **pyserial**, not `serial`. `pip install serial` installs an unrelated package that breaks things (see Troubleshooting).
- Mac with Homebrew Python says **"externally-managed-environment"**? Make a small private environment in the repo folder once:
  ```
  python3 -m venv .venv
  source .venv/bin/activate
  pip install pyserial
  ```
  Afterwards, run `source .venv/bin/activate` each time you open a new Terminal before logging.

### 3. Check the USB driver

If the Arduino IDE can upload to the Nano, the driver is fine: skip this step.

Our Nano uses a **CH340** USB chip. Windows 10/11 and recent macOS usually install its driver automatically. If no port shows up when you plug it in, install the CH340 driver from the chip maker (WCH) and re-plug.

### 4. Open a terminal in the repo folder

All commands below are run **from the repo root** (the `METR2800-GROUP1` folder that contains `tools/` and `logs/`).

- **Windows:** open the `METR2800-GROUP1` folder in File Explorer, click in the address bar, type `powershell` and press Enter. (Or right-click inside the folder → **Open in Terminal**.)
- **macOS:** in Terminal type `cd ` (with a space), drag the `METR2800-GROUP1` folder onto the window, press Enter.

### 5. Test it

```
python tools/capture_log.py --help
```

A list of options means everything is installed. You're done with setup.

---

## Part 2: log a V2.1 run

### Before the first log: set up the sketch for the bench

At the top of `Robot_Controller_V2_1.ino`:

```cpp
const RunMode RUN_MODE = RUN_FULL;
const bool WAIT_FOR_GO = true;     // bench: wait for 'g' instead of starting on connect
```

**Why:** connecting to the Nano **resets it**, the same as switching it off and on. With `WAIT_FOR_GO = false` the robot starts its run the moment the logger connects. With `true` it holds the start pose and waits for you to type `g`.

> **Competition:** put `WAIT_FOR_GO` back to **`false`**, or the robot will sit waiting for a `g` that never comes.

Upload the sketch in the Arduino IDE, then **close the Arduino Serial Monitor** (and Serial Plotter). Only one program can use the port at a time; this includes the sequence editor's live link.

### Every run

1. Robot in its start position, **slides fully in**, **battery on**, USB plugged in.
2. In the terminal (repo root):
   ```
   python tools/capture_log.py --label first_try
   ```
   `--label` is optional: a short note added to the file name so you can find the run later (letters, numbers, `-` and `_`).
3. You'll see `Capturing COM5 @ 115200 -> logs/run_..._first_try.csv` and the robot's start-up messages. Wait for `# WAIT_FOR_GO: holding start pose`.
4. Type **`g`** and press **Enter**. The robot runs. Each step is printed as it starts; warnings (stall, timeout…) are marked `!!`.
5. The logger stops by itself 1 second after the run finishes, prints a summary, and opens the viewer:
   ```
   Run time  8.91 s  (boot -> DONE)  ~ scored 9.0 s -> RUNTIMEscore 27.75 if all 6 deposited
   CSV    logs/run_20261015_143012_first_try.csv
   Viewer logs/run_20261015_143012_first_try.html   (double-click to open)
   ```

**To stop early:** type **`x`** + Enter (motors and servos stop where they are; the logger then finishes as normal), or press **Ctrl+C** (stops logging; everything so far is kept and the viewer is still written).

### Logging in console mode (jog, step by step, slow motion)

With `RUN_MODE = RUN_CONSOLE` the robot waits for commands and never finishes a run on its own, so add `--keep-open`:

```
python tools/capture_log.py --keep-open --label stepmode
```

Then type commands + Enter, for example:

| Type | Does |
|---|---|
| `?` | list all commands |
| `k 25` | run at 25% speed |
| `t` | step mode on/off (pauses after every step) |
| `g` | run the sequence |
| `n` | next step (in step mode) |
| `x` | stop |

Press **Ctrl+C** when you've finished to write the viewer.

---

## Reading the results

- **The `.html` file** opens the run viewer with graphs of arm, scoop and slides against time, the step timeline, warnings, and a 3D replay. Keep it in `logs/`: it loads the 3D model from `tools/`, so moving it elsewhere breaks the 3D view.
- **Compare runs:** open `tools/run_viewer.html` and drag several `.csv` files onto it.
- **Share a run:** send the `.csv` (small). The other person drops it on their `tools/run_viewer.html`.
- What the columns mean: see "Telemetry" in this folder's `README.md`.

## All options

| Option | What it does |
|---|---|
| `--port COM5` | Use this port instead of auto-detect. Mac: `/dev/cu.usbserial-…` or `/dev/cu.wchusbserial…` |
| `--label text` | Note added to the file name |
| `--keep-open` | Don't stop after the run finishes and no time limit (console mode, Slide_Test, Slide_PID_Tune) |
| `--max-seconds 60` | Stop after this long (default 180 s; unlimited with `--keep-open`) |
| `--live` | Live status line (arm, scoop, slide, PWM). Can garble what you type |
| `--no-open` | Don't open the viewer in the browser at the end |
| `--no-reset` | Try not to reset the Nano when connecting (works with some drivers only) |
| `--out folder` | Save somewhere other than `logs/` |
| `--baud 115200` | Serial speed. All our sketches use 115200 |

**Which port is the robot?** The logger finds the Nano automatically when it's the only one plugged in. If it lists several ports, use the one the Arduino IDE shows under **Tools → Port**, or Windows **Device Manager → Ports (COM & LPT)** ("USB-SERIAL CH340 (COM5)").

---

## Troubleshooting

| Message or symptom | Fix |
|---|---|
| `'python' is not recognized…` | Python isn't on PATH. Use `py` instead, or re-run the installer and tick "Add python.exe to PATH". |
| `pyserial is missing. Install it with: pip install pyserial` | Install it with the same Python you run the script with: `python -m pip install pyserial` (or `py -m pip …`). |
| `AttributeError: module 'serial' has no attribute 'Serial'` | The wrong `serial` package is installed. `python -m pip uninstall serial`, then `python -m pip install --force-reinstall pyserial`. |
| `could not open port 'COM5': PermissionError(13, 'Access is denied.')` | Another program has the port: Arduino Serial Monitor or Plotter, the sequence editor's live link, or another logger window. Close it and try again. Uploading from the Arduino IDE also needs the port: stop the logger first. |
| `No serial ports found. Is the Nano plugged in?` | Try another USB cable (some are charge-only) or another USB port. Check the CH340 driver (Part 1, step 3). |
| `Pick a port with --port. Available: …` | Several ports found. Add `--port` with the robot's port (see "Which port is the robot?"). |
| The robot starts moving as soon as you run the logger | `WAIT_FOR_GO` is `false`. Set it to `true` for bench logging. |
| Nothing happens after `g` | In `RUN_FULL`, `g` only works while the robot is waiting after start-up (`WAIT_FOR_GO = true`). After a run, just start the logger again: connecting resets the Nano. Also check the battery is on. |
| Garbled characters | Wrong speed. Use the default 115200 (don't pass `--baud`). |
| `No DONE seen` in the summary | The run was stopped early, the USB came out, or you were in console mode. The CSV is still saved. |
| The `.html` opens but the 3D twin is blank | The `.html` was moved out of `logs/`, or `tools/three.min.js` / `tools/twin_model.js` are missing. Open `tools/run_viewer.html` and drop the `.csv` on it instead. |
| Linux: `Permission denied: '/dev/ttyUSB0'` | `sudo usermod -aG dialout $USER`, then log out and back in. |
| The encoder or slide numbers drift while nothing moves | Battery off: the encoder needs battery power, USB alone isn't enough. |
