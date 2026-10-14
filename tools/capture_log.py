#!/usr/bin/env python3
"""
Capture a robot run over USB serial.

For each run it writes:
  logs/run_YYYYMMDD_HHMMSS[_label].csv    raw telemetry (what the Nano printed)
  logs/run_YYYYMMDD_HHMMSS[_label].html   run viewer with this run embedded

Usage (from the repo root):
  pip install pyserial
  python tools/capture_log.py                    # auto-detect the Nano
  python tools/capture_log.py --port COM5 --label dump800ms
  python tools/capture_log.py --keep-open        # Slide_Test / console mode: stays open, type commands

How a test run works:
  1. Switch the robot's battery ON (servos and motor need power).
  2. Run this script. Opening the port resets the Nano (DTR). With
     WAIT_FOR_GO = false the run starts straight away (like the power switch);
     with WAIT_FOR_GO = true it waits until you type g + Enter.
  3. The script stops 1 s after the run finishes (DONE) and prints a summary.

Anything you type + Enter is sent to the robot ('x' = abort, '?' = help in
RUN_CONSOLE mode). Ctrl+C stops capturing at any time; the file is kept.
Close the Arduino IDE Serial Monitor first, only one program can own the port.
"""

import argparse
import datetime as dt
import json
import os
import sys
import threading
import time
import webbrowser

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial is missing. Install it with:  pip install pyserial")

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
VIEWER = os.path.join(HERE, "run_viewer.html")
EMBED_MARKER = "/*@@EMBEDDED_LOGS@@*/"

STATE_NAMES = ["IDLE", "DOWN", "SCOOP", "PICKUP", "TRANSFER", "BEFORE_EXTEND",
               "OUTTAKE_EXTEND", "OUTTAKE_RETRACT", "DONE"]
WARN_CODES = {"STALL", "TIMEOUT", "NO_MOTION", "DIR_MISMATCH", "WATCHDOG", "ABORT",
              "NOT_CALIBRATED", "SETTLED_SHORT", "STALL_CUT", "I2C_TIMEOUT", "NO_SERVO_DRIVER"}

# USB-serial chips found on Nanos and clones
KNOWN_VIDS = {0x1A86: "CH340", 0x0403: "FTDI", 0x2341: "Arduino", 0x10C4: "CP210x"}


def find_port():
    ports = list(serial.tools.list_ports.comports())
    likely = [p for p in ports if p.vid in KNOWN_VIDS]
    if len(likely) == 1:
        return likely[0].device
    if not ports:
        sys.exit("No serial ports found. Is the Nano plugged in?")
    print("Pick a port with --port. Available:")
    for p in ports:
        tag = KNOWN_VIDS.get(p.vid, "")
        print(f"  {p.device:10s} {tag:8s} {p.description}")
    sys.exit(1)


def stdin_sender(ser, stop):
    """Forward typed lines to the robot."""
    while not stop.is_set():
        try:
            line = sys.stdin.readline()
        except (EOFError, KeyboardInterrupt):
            return
        if not line:
            return
        try:
            cmd = line.strip()
            if cmd:
                print(f"  > {cmd}")             # echo what was sent
            ser.write((cmd + "\n").encode("ascii", "ignore"))
        except serial.SerialException:
            return


def write_viewer(csv_path, text, label):
    if not os.path.exists(VIEWER):
        return None
    with open(VIEWER, "r", encoding="utf-8") as f:
        page = f.read()
    if EMBED_MARKER not in page:
        return None
    payload = json.dumps({"name": os.path.basename(csv_path), "label": label, "text": text})
    payload = payload.replace("</", "<\\/")          # never close the <script> early
    page = page.replace(EMBED_MARKER, payload + ",", 1)
    # the 3D twin's three.js + robot model stay in tools/; point the copy at them
    rel = os.path.relpath(HERE, os.path.dirname(os.path.abspath(csv_path))).replace("\\", "/")
    for lib in ("three.min.js", "twin_model.js"):
        page = page.replace(f'src="{lib}"', f'src="{rel}/{lib}"', 1)
    html_path = os.path.splitext(csv_path)[0] + ".html"
    with open(html_path, "w", encoding="utf-8") as f:
        f.write(page)
    return html_path


def main():
    ap = argparse.ArgumentParser(description="Capture robot telemetry to CSV + viewer HTML")
    ap.add_argument("--port", help="e.g. COM5 or /dev/ttyUSB0 (auto-detect if omitted)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", default=os.path.join(REPO, "logs"), help="output folder")
    ap.add_argument("--label", default="", help="short note added to the file name")
    ap.add_argument("--keep-open", action="store_true",
                    help="don't stop after DONE and no time limit (Slide_Test, RUN_CONSOLE)")
    ap.add_argument("--max-seconds", type=float, default=None,
                    help="stop after this long (default 180 s, unlimited with --keep-open)")
    ap.add_argument("--live", action="store_true",
                    help="show a live status line (can garble what you type)")
    ap.add_argument("--no-open", action="store_true",
                    help="don't open the viewer in the browser at the end")
    ap.add_argument("--no-reset", action="store_true",
                    help="try not to reset the Nano on connect (driver-dependent)")
    args = ap.parse_args()

    if args.max_seconds is None:
        args.max_seconds = float("inf") if args.keep_open else 180.0
    port = args.port or find_port()
    os.makedirs(args.out, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    label = "".join(c if c.isalnum() or c in "-_" else "_" for c in args.label)
    name = f"run_{stamp}" + (f"_{label}" if label else "")
    csv_path = os.path.join(args.out, name + ".csv")

    ser = serial.Serial()
    ser.port = port
    ser.baudrate = args.baud
    ser.timeout = 0.1
    if args.no_reset:
        ser.dtr = False
    ser.open()

    print(f"Capturing {port} @ {args.baud} -> {os.path.relpath(csv_path)}")
    print("Type a command + Enter to send it ('x' aborts). Ctrl+C to stop.")
    print("Press Ctrl+C when finished: the graph viewer (.html) is written and opened.\n")

    stop = threading.Event()
    threading.Thread(target=stdin_sender, args=(ser, stop), daemon=True).start()

    lines = [f"#CAPTURE,{dt.datetime.now().isoformat(timespec='seconds')},{port},{args.label}"]
    t_start = time.time()
    done_at = None
    last_status = 0.0
    run_ms = None
    buf = b""

    try:
        with open(csv_path, "w", encoding="utf-8", newline="\n") as f:
            f.write(lines[0] + "\n")
            while True:
                chunk = ser.read(512)
                if chunk:
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        line = raw.decode("ascii", "replace").strip()
                        if not line:
                            continue
                        lines.append(line)
                        f.write(line + "\n")
                        kind = line[0]
                        if kind == "D":
                            if not args.live:
                                continue
                            now = time.time()
                            if now - last_status > 0.25:
                                last_status = now
                                p = line.split(",")
                                if len(p) >= 17:
                                    st = STATE_NAMES[int(p[2])] if p[2].isdigit() and int(p[2]) < len(STATE_NAMES) else p[2]
                                    print(f"\r  t={int(p[1])/1000:7.2f}s  {st:16s} arm={p[5]:>4s}  scoop={p[7]:>4s}  "
                                          f"slide={p[11]:>6s}/{p[10]:>6s}  pwm={p[13]:>4s}   ", end="", flush=True)
                        elif kind == "S":
                            p = line.split(",")
                            print(f"\r  [{int(p[1])/1000:7.2f}s] -> {p[2]:<40s}")
                        elif kind == "E":
                            p = line.split(",")
                            mark = "  !!" if p[2] in WARN_CODES else "    "
                            print(f"\r{mark}[{int(p[1])/1000:7.2f}s] {p[2]} {p[3] if len(p) > 3 else ''}{' ' * 30}")
                        elif kind == "R":
                            p = line.split(",")
                            run_ms = int(p[2])
                            done_at = time.time()
                        elif line.startswith("#STATES,"):
                            STATE_NAMES[:] = line.split(",")[1:]   # sketch's own state names (e.g. Slide_Test)
                        elif kind == "#" and not line.startswith("#CFG"):
                            print(f"\r  {line}{' ' * 20}")
                if done_at and not args.keep_open and time.time() - done_at > 1.0:
                    break
                if time.time() - t_start > args.max_seconds:
                    print("\nStopped: --max-seconds reached.")
                    break
    except KeyboardInterrupt:
        print("\nStopped by Ctrl+C.")
    finally:
        stop.set()
        ser.close()

    text = "\n".join(lines) + "\n"
    html_path = write_viewer(csv_path, text, args.label)

    print("\n" + "=" * 60)
    if run_ms is not None:
        scored = -(-run_ms // 500) * 0.5      # rules round UP to the next 0.5 s
        print(f"Run time  {run_ms/1000:.2f} s  (boot -> DONE)  ~ scored {scored:.1f} s "
              f"-> RUNTIMEscore {(120 - scored) * 0.25:.2f} if all 6 deposited")
    else:
        print("No DONE seen (aborted, unplugged, or console mode).")
    warns = [l for l in lines if l.startswith("E,") and l.split(",")[2] in WARN_CODES]
    for w in warns:
        print("  warning:", w)
    print(f"CSV    {os.path.relpath(csv_path)}")
    if html_path:
        print(f"Viewer {os.path.relpath(html_path)}   (double-click to open)")
        if not args.no_open:
            webbrowser.open("file:///" + os.path.abspath(html_path).replace("\\", "/"))
    else:
        print(f"Viewer: open tools/run_viewer.html and drop the CSV on it")


if __name__ == "__main__":
    main()
