#!/usr/bin/env python3
"""
gary_lidar_map.py -- persistent, world-frame (Cartesian) map built from
Gary's LiDAR + dead-reckoning odometry, instead of the ego-centric polar
"radar sweep" view in gary_lidar_view.py.

Reads "SCAN:<x_cm>,<y_cm>,<theta_deg>;<d0>,<d1>,...,<d359>\n" lines that
GARY_v3_odom.ino / GARY_v3_wifi.ino print (pose first, then one distance
value per degree in the LiDAR's own frame, 0 = no reading).

For each scan, every point is rotated by Gary's heading and translated by
Gary's position at the moment of that scan, then added to a running point
cloud -- so the map keeps building up as Gary walks around, rather than
resetting every frame.

IMPORTANT CAVEATS (read before trusting what you see):
  - Odometry here is DEAD RECKONING: it integrates the same send_vy /
    send_omega values used to drive Gary's legs, with two placeholder
    calibration constants (ODOM_VY_SCALE / ODOM_OMEGA_SCALE, both 1.0
    right now) in the .ino. Until those are calibrated against a real
    known-distance / known-turn test, position drifts and the map will
    slowly warp the longer Gary walks. Rotating in place a lot will drift
    it fastest.
  - Only process_controller() (manual/RC driving) feeds odometry right
    now -- autonomous movement (exec_walk/exec_turn from voice commands
    etc.) does NOT yet update odom_x/odom_y/odom_theta_deg.
  - LiDAR angle 0 is assumed to line up with Gary's own heading reference
    (theta_deg = 0 at boot). If the LiDAR was mounted rotated relative to
    Gary's front, add a fixed offset to LIDAR_MOUNT_OFFSET_DEG below.

REQUIREMENTS (install once):
    pip install pyserial matplotlib

USAGE (USB serial):
    python gary_lidar_map.py COM5
    python gary_lidar_map.py /dev/ttyUSB0

USAGE (WiFi, once GARY_v3_wifi.ino has your network credentials filled in):
    python gary_lidar_map.py tcp:192.168.1.42:3333

    If you don't know the port name, run with no arguments and the script
    will list every serial port it can see.

Controls:
    Press 'c' while the plot window is focused to clear the accumulated
    map and start fresh (handy after odometry has drifted too far).
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

# If the LiDAR's own 0-degree mark isn't aligned with Gary's forward
# heading reference, add the offset here (degrees).
LIDAR_MOUNT_OFFSET_DEG = 0.0

# Cap on stored points so a long session doesn't grow memory forever.
# Oldest points are dropped first once this is exceeded.
MAX_POINTS = 200_000

# Distance colormap range, in mm -- just for coloring, not an axis limit
# (the map axis auto-expands to fit everything accumulated so far).
COLOR_RANGE_MM = 4000


def list_ports_and_exit():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found. Is Gary plugged in?")
    else:
        print("Available serial ports:")
        for p in ports:
            print(f"  {p.device}  -  {p.description}")
    print("\nRun again as: python gary_lidar_map.py <PORT or tcp:HOST:PORT>")
    sys.exit(1)


def parse_scan_line(line):
    """Parse 'SCAN:x_cm,y_cm,theta_deg;d0,d1,...,d359'. Returns
    (pose_x_cm, pose_y_cm, pose_theta_deg, local_angles_deg, dists_mm) or
    None if the line is malformed / has no valid points."""
    if not line.startswith("SCAN:"):
        return None
    body = line[len("SCAN:"):].strip()
    if ";" not in body:
        return None
    pose_part, scan_part = body.split(";", 1)
    try:
        px, py, ptheta = (float(v) for v in pose_part.split(","))
    except ValueError:
        return None
    if not scan_part:
        return None
    angles = []
    dists = []
    for i, tok in enumerate(scan_part.split(",")):
        try:
            d = int(tok)
        except ValueError:
            continue
        if d <= 0:
            continue
        angles.append(i)
        dists.append(d)
    if not angles:
        return None
    return px, py, ptheta, np.array(angles, dtype=float), np.array(dists, dtype=float)


def main():
    if len(sys.argv) != 2:
        list_ports_and_exit()

    port = sys.argv[1]

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

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect("equal")
    ax.set_xlabel("x (cm)")
    ax.set_ylabel("y (cm)")
    ax.set_title(
        "Gary LiDAR -- accumulated world-frame map (dead-reckoning odometry)\n"
        "Position WILL drift over a long walk -- see caveats in the script header.\n"
        "Press 'c' to clear.",
        fontsize=9,
    )
    ax.grid(True, alpha=0.3)

    cmap = matplotlib.colormaps["plasma_r"]
    scatter = ax.scatter([], [], s=3, zorder=3)

    # Gary's current position marker + heading arrow.
    gary_marker, = ax.plot([], [], marker="o", markersize=10, color="red", zorder=5)
    heading_arrow = [ax.annotate("", xy=(0, 0), xytext=(0, 0),
                                  arrowprops=dict(arrowstyle="->", color="red", lw=2), zorder=6)]

    status_text = ax.text(
        0.5, -0.08, "waiting for data...", transform=ax.transAxes,
        ha="center", fontsize=9, color="gray",
    )

    # Accumulated map: world-frame x/y in cm, plus distance (for coloring).
    cloud = {"x": np.array([]), "y": np.array([]), "d": np.array([])}
    last_pose = {"x": 0.0, "y": 0.0, "theta": 0.0, "have": False}
    scan_count = [0]
    bounds = {"xmin": -50.0, "xmax": 50.0, "ymin": -50.0, "ymax": 50.0}

    def on_key(event):
        if event.key == "c":
            cloud["x"] = np.array([])
            cloud["y"] = np.array([])
            cloud["d"] = np.array([])
            print("Map cleared.")

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_frame):
        got_new = False
        while ser.in_waiting:
            try:
                raw = ser.readline().decode("utf-8", errors="ignore").strip()
            except Exception:
                continue
            parsed = parse_scan_line(raw)
            if parsed is None:
                continue
            px, py, ptheta, local_angles_deg, dists_mm = parsed
            last_pose.update(x=px, y=py, theta=ptheta, have=True)

            # Rotate each local-frame point (LiDAR's own angle reference)
            # by Gary's heading, then translate by Gary's position.
            # Distances are in mm; pose x/y are in cm -- convert dists to
            # cm so the whole map is in one consistent unit.
            world_angles_rad = np.radians(local_angles_deg + ptheta + LIDAR_MOUNT_OFFSET_DEG)
            dists_cm = dists_mm / 10.0
            wx = px + dists_cm * np.cos(world_angles_rad)
            wy = py + dists_cm * np.sin(world_angles_rad)

            cloud["x"] = np.concatenate([cloud["x"], wx])
            cloud["y"] = np.concatenate([cloud["y"], wy])
            cloud["d"] = np.concatenate([cloud["d"], dists_mm])

            # Trim to the most recent MAX_POINTS if we've grown past the cap.
            if len(cloud["x"]) > MAX_POINTS:
                excess = len(cloud["x"]) - MAX_POINTS
                cloud["x"] = cloud["x"][excess:]
                cloud["y"] = cloud["y"][excess:]
                cloud["d"] = cloud["d"][excess:]

            scan_count[0] += 1
            got_new = True

        if got_new and len(cloud["x"]) > 0:
            scatter.set_offsets(np.column_stack([cloud["x"], cloud["y"]]))
            norm = mcolors.Normalize(vmin=0, vmax=COLOR_RANGE_MM)
            scatter.set_color(cmap(norm(cloud["d"])))

            # Recompute axis bounds to fit everything, with a little padding.
            pad = 30.0
            bounds["xmin"] = min(cloud["x"].min(), last_pose["x"]) - pad
            bounds["xmax"] = max(cloud["x"].max(), last_pose["x"]) + pad
            bounds["ymin"] = min(cloud["y"].min(), last_pose["y"]) - pad
            bounds["ymax"] = max(cloud["y"].max(), last_pose["y"]) + pad
            ax.set_xlim(bounds["xmin"], bounds["xmax"])
            ax.set_ylim(bounds["ymin"], bounds["ymax"])

        if last_pose["have"]:
            gary_marker.set_data([last_pose["x"]], [last_pose["y"]])
            heading_rad = np.radians(last_pose["theta"])
            arrow_len = 20.0  # cm
            tip_x = last_pose["x"] + arrow_len * np.cos(heading_rad)
            tip_y = last_pose["y"] + arrow_len * np.sin(heading_rad)
            heading_arrow[0].xy = (tip_x, tip_y)
            heading_arrow[0].xyann = (last_pose["x"], last_pose["y"])

        if got_new:
            status_text.set_text(
                f"scans: {scan_count[0]}  |  points: {len(cloud['x'])}  |  "
                f"pose: ({last_pose['x']:.0f}, {last_pose['y']:.0f}) cm @ {last_pose['theta']:.0f} deg"
            )
        return scatter, gary_marker, status_text

    ani = animation.FuncAnimation(fig, update, interval=150, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()

    ser.close()


if __name__ == "__main__":
    main()
