#!/usr/bin/env python3
"""
gary_lidar_view.py -- live radar view of Gary's STL-19P LiDAR scan.

Reads "SCAN:<d0>,<d1>,...,<d359>\n" lines that GARY_v3_lidarview2.ino prints
over the same USB-serial link as the regular debug output (full 360-degree
resolution, one value per degree -- see the "Full-scan dump" comment block
in the .ino).

Features:
  - Full 360-point resolution (one dot per degree).
  - Fixed range (see MAX_RANGE_MM below, or pass a value on the command
    line) with matplotlib's default concentric ring grid.
  - Persistence / afterglow: points from the last PERSIST_SECONDS stay on
    screen and fade out, instead of the plot being wiped every 400ms. This
    is NOT a real map (no odometry -- points are in the LiDAR's own frame),
    so it only looks like a coherent room if Gary is roughly stationary
    while scanning (e.g. turning in place). If Gary walks, old points will
    visibly "smear" rather than lining up -- that's expected, not a bug.
  - Distance colormap (near=warm/yellow, far=cool/purple) combined with
    the persistence fade (newest = fully opaque, oldest = nearly invisible).
  - A reference line marking the LiDAR's own 0 degree (see the note in
    Gary_lidar.h -- this is whichever way the sensor faced at boot, not
    necessarily Gary's front).

REQUIREMENTS (install once):
    pip install pyserial matplotlib

BEFORE RUNNING (USB mode):
    Close the Arduino IDE's Serial Monitor / Plotter first. Only one
    program can hold a COM port open at a time -- if the Serial Monitor
    has it, this script will fail to connect (or silently get nothing).

USAGE (USB serial):
    python gary_lidar_view.py COM5              (Windows)
    python gary_lidar_view.py /dev/ttyUSB0      (Linux)
    python gary_lidar_view.py /dev/cu.usbserial-XXXX   (macOS)
    python gary_lidar_view.py COM5 6000         (start range at 6m instead of default)

    If you don't know the port name, run with no arguments and the script
    will list every serial port it can see.

USAGE (WiFi, once GARY_v3_wifi.ino is flashed with your network's
credentials filled in -- Gary prints his IP over Serial once at boot):
    python gary_lidar_view.py tcp:192.168.1.42:3333
    python gary_lidar_view.py tcp:192.168.1.42:3333 6000
"""

import sys
import time
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.colors as mcolors
import numpy as np
import serial.tools.list_ports  # only used for list_ports_and_exit()

from gary_lidar_io import open_source, set_scanner_mode

BAUD = 921600   # matches GARY_SERIAL_BAUD in GARY_v3_autoscan.ino

# Fixed radius axis in millimeters. Override with a second command-line
# argument (e.g. "python gary_lidar_view.py COM15 6000" for 6m) if a
# room needs more or less than this.
MAX_RANGE_MM = 4000

# How long a point stays on screen before fully fading out, in seconds.
# At a 400ms scan rate this keeps roughly the last 3-4 scans visible.
PERSIST_SECONDS = 1.5


def list_ports_and_exit():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found. Is Gary plugged in?")
    else:
        print("Available serial ports:")
        for p in ports:
            print(f"  {p.device}  -  {p.description}")
    print("\nRun again as: python gary_lidar_view.py <PORT> [max_range_mm]")
    sys.exit(1)


def parse_scan_line(line):
    """Turn a 'SCAN:123,456,...' line (one value per degree, index i ==
    degree i) into (angles_deg, distances_mm) arrays, dropping zero entries
    (0 = no data / invalid reading, per the driver)."""
    if not line.startswith("SCAN:"):
        return None
    body = line[len("SCAN:"):].strip()
    if not body:
        return None
    parts = body.split(",")
    angles = []
    dists = []
    for i, tok in enumerate(parts):
        try:
            d = int(tok)
        except ValueError:
            continue
        if d <= 0:
            continue
        angles.append(i)  # full resolution: index i -> degree i
        dists.append(d)
    if not angles:
        return None
    return np.radians(angles), np.array(dists, dtype=float)


def main():
    if len(sys.argv) not in (2, 3):
        list_ports_and_exit()

    port = sys.argv[1]
    rmax = float(sys.argv[2]) if len(sys.argv) == 3 else MAX_RANGE_MM

    try:
        ser = open_source(port, BAUD)
        set_scanner_mode(ser, True)   # scanner mode is OFF at boot -- ask for the SCAN: stream
    except Exception as e:
        print(f"Could not open {port}: {e}")
        if port.lower().startswith("tcp:"):
            print("(Is Gary powered on, connected to WiFi, and on the same network? "
                  "Check the IP Gary printed over Serial at boot.)")
        else:
            print("(Is the Arduino Serial Monitor still open? Close it and retry.)")
        sys.exit(1)

    fig = plt.figure(figsize=(7.5, 7.5))
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N")   # 0 deg pointing "up" on screen
    ax.set_theta_direction(-1)        # clockwise, like a radar sweep
    ax.set_rlim(0, rmax)              # fixed range; matplotlib draws its own default rings

    ax.set_title(
        "Gary LiDAR -- live scan (afterglow ~%.1fs)\n"
        "0deg (cyan line) is whichever way the LiDAR faced at boot -- not\n"
        "necessarily Gary's front. See Gary_lidar.h for calibrating an offset."
        % PERSIST_SECONDS,
        fontsize=9,
    )

    # Reference line marking the LiDAR's own 0-degree direction.
    ax.plot([0, 0], [0, rmax * 0.15], color="cyan", lw=2, zorder=5)

    cmap = matplotlib.colormaps["plasma_r"]  # near = bright/yellow, far = dark/purple
    scatter = ax.scatter([], [], s=6, zorder=3)

    status_text = ax.text(
        0.5, -0.10, "waiting for data...", transform=ax.transAxes,
        ha="center", fontsize=9, color="gray",
    )

    # Persistence buffer: list of [angle_rad, dist_mm, timestamp]
    points = {"angle": np.array([]), "dist": np.array([]), "t": np.array([])}
    frame_count = [0]

    def update(_frame):
        now = time.time()
        got_new = False
        while ser.in_waiting:
            try:
                raw = ser.readline().decode("utf-8", errors="ignore").strip()
            except Exception:
                continue
            parsed = parse_scan_line(raw)
            if parsed is not None:
                angles, dists = parsed
                ts = np.full(len(angles), now)
                points["angle"] = np.concatenate([points["angle"], angles])
                points["dist"] = np.concatenate([points["dist"], dists])
                points["t"] = np.concatenate([points["t"], ts])
                frame_count[0] += 1
                got_new = True

        # Drop anything older than the persistence window.
        cutoff = now - PERSIST_SECONDS
        keep = points["t"] >= cutoff
        for k in points:
            points[k] = points[k][keep]

        if len(points["angle"]) > 0:
            scatter.set_offsets(np.column_stack([points["angle"], points["dist"]]))
            # Color by distance...
            norm = mcolors.Normalize(vmin=0, vmax=rmax)
            colors = cmap(norm(points["dist"]))
            # ...then fade alpha by age (newest = 1.0, oldest = ~0.1).
            age_frac = np.clip((now - points["t"]) / PERSIST_SECONDS, 0, 1)
            colors[:, 3] = 1.0 - 0.9 * age_frac
            scatter.set_color(colors)

        if got_new:
            status_text.set_text(
                f"frames: {frame_count[0]}  |  live points: {len(points['angle'])}  |  range: {rmax/1000:.1f}m"
            )
        return scatter, status_text

    ani = animation.FuncAnimation(fig, update, interval=150, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()

    ser.close()


if __name__ == "__main__":
    main()
