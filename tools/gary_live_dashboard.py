#!/usr/bin/env python3
"""
gary_live_dashboard.py

Live 3D nod-scan point cloud + CSI/AUTOSCAN dashboard (matplotlib window).
Connects to Gary the same way gary_lidar_view.py / gary_lidar_map.py do (see
gary_lidar_io.py), reads his streamed SCAN:/NOD:/AUTOSCAN/RSSI: lines as they
arrive, and updates two live panels:

  Left:  3D point cloud, accumulating every NOD: slice all session long
         (persistent map -- nothing is cleared between scans). Points captured
         during a CSI-triggered scan are colored differently (orange/red) from
         points captured during a manually-run scan (cyan), so you can see at a
         glance which parts of the room were mapped *because* AUTOSCAN caught
         something there.
  Right: live RSSI strip chart -- baseline (blue) vs. instantaneous reading
         (gray), with vertical red lines marking TRIP events.

Usage:
    python gary_live_dashboard.py --source tcp:192.168.1.42:3333
    python gary_live_dashboard.py --source COM15        (USB serial instead)

Prefer a browser and a real link? See gary_web_dashboard.py -- same data, but
Plotly gives correct camera behavior for free and you get a URL you can open
from your phone.

SCANNER MODE
Gary boots with the scan stream OFF (it is the single biggest consumer of his
firmware loop time). This script sends '1' on connect to switch it on and '0'
on exit to quiet him back down, so you don't have to touch the controller.
Requires firmware with poll_console(); older builds never read the WiFi client,
so they accept your connection and then send nothing at all.

CAMERA CONTROLS (3D panel)
    left-drag        orbit / rotate
    scroll wheel     zoom in / out
    right-drag       pan (shifts the view center)
    r                reset camera to the default angle
    f                re-fit the axes around the current point cloud

The camera is now PRESERVED across redraws. The previous version called
ax.clear() on every frame, which silently reset elev/azim and the axis limits
two or three times a second -- that is why the view fought back when you tried
to move it, and the wheel did nothing because matplotlib's 3D axes have no
wheel zoom at all by default. Artists are now updated in place, and only 'f' /
'r' (or the very first points arriving) ever touch the limits.

POINT CLOUD CLEANUP
Raw returns include zeros (no echo), out-of-range garbage, and large numbers of
near-duplicate points from overlapping sweeps. Three filters, all tunable:
    --min-mm / --max-mm    hard range gate (default 120..6000 mm)
    --voxel-mm             voxel downsample grid, one point per cell (default 40)
    --max-points           hard cap on plotted points, oldest dropped first
The voxel grid is what keeps redraws fast: without it the scatter grows without
bound and the frame rate collapses after a few minutes of scanning. A single
specular return off a window will also blow the auto-fitted axes out to nonsense
and squash the whole cloud into a dot -- that is what --max-mm is for.

CAVEAT -- odometry / multi-location mapping:
Every point is treated as if Gary is at world origin (0,0,0) facing +X for the
whole session. Accurate while he stays in one spot (already true today: he must
stand still for CSI to mean anything, and NODSCAN tilts in place rather than
walking), so repeated scans from one sitting stack correctly with no smearing.
NOT accurate if he relocates between scans and you want one merged map spanning
several spots -- that needs the odometry actually streamed out, which it isn't.
"""

import argparse
import re
import sys
import time
from collections import deque

import numpy as np
import matplotlib
import matplotlib.animation
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from gary_lidar_io import open_source, set_scanner_mode
import nod_pointcloud as npc

NOD_RE = npc.NOD_RE
RSSI_RE = re.compile(r"^RSSI:(-?\d+),(-?\d+(?:\.\d+)?)$")
STATUS_RE = re.compile(r">>> AUTOSCAN status \| phase=(\w+) baseline=(-?\d+\.\d+)dBm samples=(\d+) triggered=(\d+)")
TRIP_RE = re.compile(r">>> AUTOSCAN: TRIP")
TRIG_START_RE = re.compile(r">>> AUTOSCAN: triggered scan #(\d+) starting")
TRIG_DONE_RE = re.compile(r">>> AUTOSCAN: triggered scan complete, resuming watch")
SWEEP_DONE_RE = re.compile(r">>> NODSCAN: sweep complete")

RSSI_HISTORY_LEN = 400  # ~2min at the firmware's 300ms stream rate
DEFAULT_ELEV, DEFAULT_AZIM = 25.0, -60.0


class PointBucket:
    """Voxel-downsampled, capped accumulator for 3D points.

    Keyed by integer voxel coordinate so re-scanning the same wall from the same
    spot doesn't pile up thousands of coincident points. Dict insertion order is
    preserved, which is what lets us drop the OLDEST voxels when the cap is hit
    rather than throwing away the newest data."""

    def __init__(self, voxel_mm, max_points):
        self.voxel_mm = float(voxel_mm)
        self.max_points = int(max_points)
        self.cells = {}

    def add(self, pts):
        if len(pts) == 0:
            return
        keys = np.floor(np.asarray(pts) / self.voxel_mm).astype(np.int64)
        for key, p in zip(map(tuple, keys), pts):
            if key not in self.cells:
                self.cells[key] = (float(p[0]), float(p[1]), float(p[2]))
        overflow = len(self.cells) - self.max_points
        if overflow > 0:
            for key in list(self.cells.keys())[:overflow]:
                del self.cells[key]

    def array(self):
        if not self.cells:
            return np.zeros((0, 3))
        return np.array(list(self.cells.values()), dtype=float)

    def __len__(self):
        return len(self.cells)


class DashboardState:
    def __init__(self, args):
        self.args = args
        self.pts_normal = PointBucket(args.voxel_mm, args.max_points)
        self.pts_triggered = PointBucket(args.voxel_mm, args.max_points)
        self.in_triggered_scan = False
        self.scans_seen = 0
        self.dropped = 0
        self.last_data_at = None

        self.rssi_t = deque(maxlen=RSSI_HISTORY_LEN)
        self.rssi_v = deque(maxlen=RSSI_HISTORY_LEN)
        self.baseline_v = deque(maxlen=RSSI_HISTORY_LEN)
        self.trip_times = deque(maxlen=50)
        self.t0 = time.time()

        self.phase = "?"
        self.baseline = None
        self.samples = None
        self.triggered_count = 0

    def clean(self, pts):
        if len(pts) == 0:
            return pts
        r = np.linalg.norm(pts, axis=1)
        keep = (r >= self.args.min_mm) & (r <= self.args.max_mm)
        self.dropped += int((~keep).sum())
        return pts[keep]

    def handle_line(self, line):
        line = line.strip()
        if not line:
            return

        m = NOD_RE.match(line)
        if m:
            pitch = float(m.group(1))
            roll = float(m.group(2)) if m.group(2) is not None else 0.0
            try:
                dists = [int(p) for p in m.group(3).split(",")]
            except ValueError:
                return
            if len(dists) != 360:
                dists = (dists + [0] * 360)[:360]
            pts = self.clean(npc.scans_to_pointcloud([(pitch, roll, dists)]))
            bucket = self.pts_triggered if self.in_triggered_scan else self.pts_normal
            bucket.add(pts)
            self.last_data_at = time.time()
            return

        m = RSSI_RE.match(line)
        if m:
            self.rssi_t.append(time.time() - self.t0)
            self.rssi_v.append(int(m.group(1)))
            self.baseline_v.append(float(m.group(2)))
            self.last_data_at = time.time()
            return

        m = STATUS_RE.search(line)
        if m:
            self.phase, self.baseline = m.group(1), float(m.group(2))
            self.samples, self.triggered_count = int(m.group(3)), int(m.group(4))
            return

        if TRIP_RE.search(line):
            self.trip_times.append(time.time() - self.t0)
            return
        if TRIG_START_RE.search(line):
            self.in_triggered_scan = True
            return
        if TRIG_DONE_RE.search(line) or SWEEP_DONE_RE.search(line):
            self.in_triggered_scan = False
            self.scans_seen += 1
            return

    def all_points(self):
        pn, pt = self.pts_normal.array(), self.pts_triggered.array()
        if len(pn) and len(pt):
            return np.vstack([pn, pt])
        return pn if len(pn) else pt


class Camera:
    """Owns the 3D axis limits so redraws never stomp on the user's view.

    matplotlib 3D gives you left-drag orbit for free but NOT wheel zoom, and
    nothing survives if the axis gets cleared every frame. So: keep our own
    center + radius, re-apply it each redraw, and drive it from wheel and
    right-drag events."""

    def __init__(self, ax):
        self.ax = ax
        self.center = np.zeros(3)
        self.radius = 2000.0
        self.fitted = False
        self.pan_start = None

    def fit(self, pts):
        if len(pts) == 0:
            return
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        self.center = (lo + hi) / 2.0
        self.radius = max(float(np.max(hi - lo)) / 2.0, 250.0) * 1.1
        self.fitted = True

    def apply(self):
        c, r = self.center, self.radius
        self.ax.set_xlim(c[0] - r, c[0] + r)
        self.ax.set_ylim(c[1] - r, c[1] + r)
        self.ax.set_zlim(c[2] - r, c[2] + r)
        self.ax.set_box_aspect((1, 1, 1))

    def zoom(self, factor):
        self.radius = float(np.clip(self.radius * factor, 100.0, 50000.0))

    def pan_pixels(self, dx, dy):
        az = np.radians(self.ax.azim)
        el = np.radians(self.ax.elev)
        right = np.array([-np.sin(az), np.cos(az), 0.0])
        up = np.array([-np.cos(az) * np.sin(el), -np.sin(az) * np.sin(el), np.cos(el)])
        scale = self.radius / 300.0
        self.center = self.center - right * dx * scale - up * dy * scale

    def reset(self):
        self.ax.view_init(elev=DEFAULT_ELEV, azim=DEFAULT_AZIM)


def compass_spokes(ax, spoke):
    ax.plot([0, spoke], [0, 0], [0, 0], color="red", linewidth=3)
    ax.text(spoke, 0, 0, "FWD", color="red", fontsize=9, fontweight="bold")
    ax.plot([0, 0], [0, spoke], [0, 0], color="green", linewidth=3)
    ax.text(0, spoke, 0, "LEFT", color="green", fontsize=9, fontweight="bold")
    ax.plot([0, 0], [0, 0], [0, spoke], color="blue", linewidth=3)
    ax.text(0, 0, spoke, "UP", color="blue", fontsize=9, fontweight="bold")
    ax.scatter([0], [0], [0], s=150, c="red", marker="^", edgecolors="black", depthshade=False)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", required=True, help="tcp:HOST:PORT (e.g. tcp:192.168.1.42:3333) or a serial port")
    ap.add_argument("--redraw-ms", type=int, default=400, help="how often to redraw the plots (ms)")
    ap.add_argument("--min-mm", type=float, default=120.0, help="drop returns closer than this (Gary's own body)")
    ap.add_argument("--max-mm", type=float, default=6000.0, help="drop returns farther than this (specular junk)")
    ap.add_argument("--voxel-mm", type=float, default=40.0, help="voxel downsample grid size in mm; 0 disables")
    ap.add_argument("--max-points", type=int, default=120000, help="hard cap on plotted points (oldest dropped)")
    ap.add_argument("--no-scanner-request", action="store_true", help="don't ask Gary to enable scanner mode on connect")
    args = ap.parse_args()
    if args.voxel_mm <= 0:
        args.voxel_mm = 1e-6

    print("Connecting to %s ..." % args.source)
    try:
        src = open_source(args.source)
    except Exception as e:
        print("Failed to connect: %s" % e, file=sys.stderr)
        sys.exit(1)
    if not args.no_scanner_request:
        set_scanner_mode(src, True)
        print("Requested scanner mode ON.")
    print("Connected. left-drag rotate | wheel zoom | right-drag pan | r reset | f fit")

    state = DashboardState(args)
    buf = b""

    fig = plt.figure(figsize=(14, 7))
    ax3d = fig.add_subplot(121, projection="3d")
    ax_rssi = fig.add_subplot(122)
    fig.suptitle("Gary -- live nod-scan point cloud + AUTOSCAN presence monitor")

    cam = Camera(ax3d)
    cam.reset()

    sc_normal = ax3d.scatter([], [], [], s=1.5, c="tab:cyan", alpha=0.55,
                             label="manual/baseline", depthshade=False)
    sc_trig = ax3d.scatter([], [], [], s=2.5, c="orangered", alpha=0.9,
                           label="CSI-triggered", depthshade=False)
    ax3d.set_xlabel("X forward (mm)")
    ax3d.set_ylabel("Y left (mm)")
    ax3d.set_zlabel("Z up (mm)")
    ax3d.legend(loc="upper left", fontsize=8)
    compass_spokes(ax3d, 400.0)
    cam.apply()

    def on_scroll(event):
        if event.inaxes is not ax3d:
            return
        cam.zoom(0.85 if event.button == "up" else 1.0 / 0.85)
        cam.apply()
        fig.canvas.draw_idle()

    def on_press(event):
        if event.inaxes is ax3d and event.button == 3:
            cam.pan_start = (event.x, event.y)

    def on_motion(event):
        if cam.pan_start is None or event.inaxes is not ax3d:
            return
        x0, y0 = cam.pan_start
        cam.pan_pixels(event.x - x0, event.y - y0)
        cam.pan_start = (event.x, event.y)
        cam.apply()
        fig.canvas.draw_idle()

    def on_release(event):
        cam.pan_start = None

    def on_key(event):
        if event.key == "r":
            cam.reset()
            cam.apply()
            fig.canvas.draw_idle()
        elif event.key == "f":
            cam.fit(state.all_points())
            cam.apply()
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect("scroll_event", on_scroll)
    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("motion_notify_event", on_motion)
    fig.canvas.mpl_connect("button_release_event", on_release)
    fig.canvas.mpl_connect("key_press_event", on_key)

    def pump_lines():
        nonlocal buf
        while getattr(src, "in_waiting", False):
            chunk = src.readline()
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    state.handle_line(line.decode("utf-8", errors="replace"))
                except Exception:
                    pass

    def redraw(_frame):
        pump_lines()
        pn = state.pts_normal.array()
        pt = state.pts_triggered.array()

        if not cam.fitted and (len(pn) or len(pt)):
            cam.fit(state.all_points())
            cam.apply()

        sc_normal._offsets3d = (pn[:, 0], pn[:, 1], pn[:, 2]) if len(pn) else ([], [], [])
        sc_trig._offsets3d = (pt[:, 0], pt[:, 1], pt[:, 2]) if len(pt) else ([], [], [])

        if state.last_data_at is None:
            link = "NO DATA YET -- scanner mode off? (touchpad click, or 's' on serial)"
        else:
            age = time.time() - state.last_data_at
            link = ("last frame %.1fs ago" % age) if age < 5 else ("STALLED -- nothing for %.0fs" % age)
        ax3d.set_title(
            "%d pts (voxel %gmm, %d dropped) | %d scans (%d triggered) | %s"
            % (len(pn) + len(pt), args.voxel_mm, state.dropped, state.scans_seen,
               state.triggered_count, link),
            fontsize=9,
        )

        ax_rssi.clear()
        if state.rssi_t:
            t = list(state.rssi_t)
            ax_rssi.plot(t, list(state.rssi_v), color="gray", linewidth=1, label="RSSI (instant)")
            ax_rssi.plot(t, list(state.baseline_v), color="tab:blue", linewidth=2, label="baseline")
        for tt in state.trip_times:
            ax_rssi.axvline(tt, color="red", linestyle="--", alpha=0.7)
        ax_rssi.set_xlabel("seconds since connect")
        ax_rssi.set_ylabel("RSSI (dBm)")
        ax_rssi.set_title(
            "phase=%s  baseline=%s  samples=%s  triggered=%s"
            % (state.phase, state.baseline, state.samples, state.triggered_count),
            fontsize=9,
        )
        ax_rssi.legend(loc="upper right", fontsize=8)
        ax_rssi.grid(True, alpha=0.3)
        return []

    ani = matplotlib.animation.FuncAnimation(fig, redraw, interval=args.redraw_ms, cache_frame_data=False)
    plt.tight_layout()
    try:
        plt.show()
    finally:
        if not args.no_scanner_request:
            set_scanner_mode(src, False)
        src.close()


if __name__ == "__main__":
    main()
