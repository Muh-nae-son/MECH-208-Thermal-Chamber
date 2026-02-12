import csv
import os
import sys
import time
import select
import tty
import termios
from collections import deque
from datetime import datetime

import serial
import matplotlib.pyplot as plt


# -------------------- USER SETTINGS --------------------
PORT = "/dev/cu.usbmodem113401"  # <-- run ls /dev/cu.*
BAUD = 9600                      
OUTPUT_DIR = "Arduino_logs"      
MAX_POINTS = 1500
PLOT_UPDATE_HZ = 30
# -------------------------------------------------------


def try_parse_csv_numbers(line: str):
    parts = [p.strip() for p in line.split(",")]
    if not parts or any(p == "" for p in parts):
        return None
    try:
        return [float(p) for p in parts]
    except ValueError:
        return None


def nonblocking_keypress():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def main():
    print(f"Opening {PORT} @ {BAUD} ...")
    ser = serial.Serial(PORT, BAUD, timeout=0.02)
    time.sleep(1.5)

    print("\nControls:")
    print("  r  -> start/stop recording (plot always live)")
    print("  q  -> quit\n")

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    plt.ion()
    fig, ax = plt.subplots()
    ax.set_title("Live Serial Plot (r = record, q = quit)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Value")
    ax.grid(True)

    recording = False
    record_rows = []
    record_start_t = None
    headers = None

    t_buf = deque(maxlen=MAX_POINTS)
    y_bufs = []
    lines = []

    last_draw = 0
    min_draw_dt = 1.0 / PLOT_UPDATE_HZ

    try:
        while True:

            # -------- keyboard ----------
            key = nonblocking_keypress()
            if key:
                key = key.lower()

                if key == "q":
                    print("\nQuitting...")
                    break

                if key == "r":
                    if not recording:
                        recording = True
                        record_rows = []
                        record_start_t = time.time()
                        headers = None
                        print("\n[REC] Started")
                        ax.set_title("RECORDING... (press r to stop)")
                    else:
                        recording = False
                        print("[REC] Stopped")

                        if record_rows:
                            ensure_dir(OUTPUT_DIR)
                            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                            filepath = os.path.join(
                                OUTPUT_DIR,
                                f"arduino_log_{ts}.csv"
                            )

                            with open(filepath, "w", newline="") as f:
                                w = csv.writer(f)
                                w.writerow(headers)
                                w.writerows(record_rows)

                            print(f"Saved CSV: {filepath}")

                        ax.set_title("Live Serial Plot (r = record, q = quit)")

            plt.pause(0.001)

            # -------- serial read ----------
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode(errors="replace").strip()
            vals = try_parse_csv_numbers(line)
            if vals is None:
                continue

            t_now = time.time()

            # initialize buffers on first valid line
            if not y_bufs:
                for _ in vals:
                    y_bufs.append(deque(maxlen=MAX_POINTS))

                for i in range(len(vals)):
                    (ln,) = ax.plot([], [], label=f"v{i+1}")
                    lines.append(ln)

                ax.legend(loc="upper right")

            # ignore if column count changes
            if len(vals) != len(y_bufs):
                continue

            # update plot buffers
            t_buf.append(t_now)

            for i, v in enumerate(vals):
                y_bufs[i].append(v)

            # record if recording enabled
            if recording:
                if headers is None:
                    headers = ["t"] + [f"v{i+1}" for i in range(len(vals))]
                record_rows.append(
                    [t_now - record_start_t] + vals
                )

            # redraw at capped rate
            now = time.time()
            if now - last_draw >= min_draw_dt:
                for i, ln in enumerate(lines):
                    ln.set_data(t_buf, y_bufs[i])

                if len(t_buf) > 2:
                    ax.set_xlim(t_buf[0], t_buf[-1])

                    ymin = min(min(buf) for buf in y_bufs if buf)
                    ymax = max(max(buf) for buf in y_bufs if buf)
                    if ymin == ymax:
                        ymin -= 1
                        ymax += 1
                    ax.set_ylim(ymin, ymax)

                fig.canvas.draw_idle()
                last_draw = now

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        ser.close()


if __name__ == "__main__":
    main()
