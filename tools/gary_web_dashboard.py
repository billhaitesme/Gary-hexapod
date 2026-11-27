#!/usr/bin/env python3
"""
gary_web_dashboard.py

Gary's live map, in a browser, at a link.

Runs a small HTTP server, connects to Gary (TCP over WiFi, or USB serial), and
serves a page showing:

  * the accumulating 3D point cloud -- BOTH sources:
      - NOD:  slices from a nod-scan (action 14): full 3D, tilted rings
      - SCAN: rings streamed continuously in scanner mode: one horizontal
              ring per frame, placed in WORLD frame using Gary's odometry
              pose, so the map keeps building as he walks around
  * movers: live-vs-reference change detection on the SCAN ring (see below)
  * the CSI / AUTOSCAN RSSI strip chart with TRIP markers
  * a scanner-mode toggle button (no need to reach for the controller)

    python gary_web_dashboard.py --source tcp:192.168.1.156:3333
    python gary_web_dashboard.py --source COM15 --port 8770

Then open http://localhost:8770 (opens automatically unless --no-browser).
Server binds 0.0.0.0, so the same link works from your phone on the LAN.


WHAT SCANNER MODE ACTUALLY GATES  (this bit was broken before)
Scanner mode controls the SCAN: stream -- the 360-point horizontal sweep with
pose. Until now this dashboard parsed only NOD: lines, so toggling scanner mode
appeared to do nothing at all and the cloud only ever grew during an action-14
nod-scan, then went STALLED. SCAN: frames are now parsed and plotted, which is
what makes the map build continuously.


WORLD FRAME / ODOMETRY -- READ THIS BEFORE TRUSTING THE MAP
Each SCAN: line carries "x_cm, y_cm, theta_deg" from Gary's dead-reckoning
odometry, and every ring is transformed into world frame with it, so walking
him around accumulates one shared map instead of smearing everything on top of
itself. But that odometry is DEAD RECKONING, and in the current firmware:
  * ODOM_VY_SCALE / ODOM_OMEGA_SCALE are still uncalibrated placeholders (1.0)
  * it only integrates during controller-driven motion, so it silently freezes
    during autonomous roaming
  * there is no loop closure and no scan matching
So the map WILL drift -- walk a loop and the two ends won't meet. It is good
enough to stitch a room together roughly; it is not SLAM. Calibrating those two
scale constants is the cheapest big win; ICP/scan-matching is the real fix.


MOVERS (CSI + LiDAR)
Straight about the physics: CSI on a single link is one scalar per packet. It
can tell you SOMETHING MOVED. It cannot tell you WHERE -- no bearing, no range,
nothing to triangulate with a single AP. So:
  * CSI (RSSI/TRIP) = the trigger, and a motion indicator that works even when
    something is out of LiDAR line-of-sight -- but with no position.
  * LiDAR change-detection = the locator. While Gary is stationary we build a
    per-bearing reference range from the first few SCAN rings; anything that
    later returns SIGNIFICANTLY CLOSER than the reference at that bearing is a
    mover. Consecutive bearings get clustered and reported as a centroid, and
    each mover's recent centroids are drawn as a trail.
Movers are only meaningful while he holds still: the reference resets whenever
the pose changes, because a moved robot sees a legitimately different world.
"""

import argparse
import json
import math
import re
import sys
import threading
import time
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

from gary_lidar_io import open_source, set_scanner_mode
import nod_pointcloud as npc
from gary_slam import Slam   # scan-matching pose correction (holds fixed locations)

NOD_RE = npc.NOD_RE
SCAN_RE = re.compile(r"^SCAN:(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)(?:,(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?))?;(.+)$")
RSSI_RE = re.compile(r"^RSSI:(-?\d+),(-?\d+(?:\.\d+)?)$")
STATUS_RE = re.compile(r">>> AUTOSCAN status \| phase=(\w+) baseline=(-?\d+\.\d+)dBm samples=(\d+) triggered=(\d+)")
TRIP_RE = re.compile(r">>> AUTOSCAN: TRIP")
TRIG_START_RE = re.compile(r">>> AUTOSCAN: triggered scan #(\d+) starting")
TRIG_DONE_RE = re.compile(r">>> AUTOSCAN: triggered scan complete, resuming watch")
SWEEP_DONE_RE = re.compile(r">>> NODSCAN: sweep complete")
SCANNER_RE = re.compile(r"\[SCAN\] Scanner mode: (ON|OFF)")

RSSI_HISTORY_LEN = 400

# Change detection tuning
REF_FRAMES = 6          # SCAN rings averaged into the static reference
MOVER_DELTA_MM = 300.0  # must come this much closer than reference to count
MOVER_MIN_BEARINGS = 3  # consecutive bearings needed to form a cluster
MOVER_TRAIL = 40        # centroids kept per track
POSE_EPS = 5.0          # cm/deg of pose change that invalidates the reference


class State:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.cells_normal = {}     # voxel -> (x,y,z)  nod-scan + scan rings
        self.cells_trig = {}       # voxel -> (x,y,z)  captured during CSI-triggered scan
        self.in_triggered_scan = False
        self.scans_seen = 0
        self.rings_seen = 0
        self.dropped = 0
        self.last_data_at = None
        self.connected = False
        self.error = None
        self.scanner_on = None     # None = unknown until Gary tells us

        self.pose = (0.0, 0.0, 0.0)   # x_cm, y_cm, theta_deg -- SCAN-MATCHED, not raw odom
        self.slam = Slam(mode="scanmatch")  # derives pose from the lidar (see handle_line)
        self.ref = None               # np.array(360) reference ranges, robot frame
        self.ref_stack = []
        self.ref_pose = None
        self.movers = []              # current frame's mover centroids (world mm)
        self.trail = deque(maxlen=MOVER_TRAIL)

        self.rssi_t = deque(maxlen=RSSI_HISTORY_LEN)
        self.rssi_v = deque(maxlen=RSSI_HISTORY_LEN)
        self.baseline_v = deque(maxlen=RSSI_HISTORY_LEN)
        self.trips = deque(maxlen=50)
        self.t0 = time.time()

        self.phase = "?"
        self.baseline = None
        self.samples = None
        self.triggered_count = 0

    # ---- point accumulation ------------------------------------------------
    def _add(self, cells, pts):
        if len(pts) == 0:
            return
        keys = np.floor(pts / self.args.voxel_mm).astype(np.int64)
        for key, p in zip(map(tuple, keys), pts):
            if key not in cells:
                cells[key] = (round(float(p[0]), 1), round(float(p[1]), 1), round(float(p[2]), 1))
        overflow = len(cells) - self.args.max_points
        if overflow > 0:
            for key in list(cells.keys())[:overflow]:
                del cells[key]

    def clean(self, pts):
        if len(pts) == 0:
            return pts
        r = np.linalg.norm(pts, axis=1)
        keep = (r >= self.args.min_mm) & (r <= self.args.max_mm)
        self.dropped += int((~keep).sum())
        return pts[keep]

    # ---- SCAN: ring -> world-frame points ----------------------------------
    def ring_to_world(self, dists, pose):
        """One 360-point ring, robot frame -> world frame. NOW IN 3D.

        This used to flatten every ring onto z = lidar_z, and that was a bug, not
        a limitation. The LiDAR is bolted to Gary's body: when he pitches or rolls,
        the scan plane tilts with him, so a ring taken at 15 degrees of pitch is
        sampling a genuinely different slice of the room than a level one. We were
        throwing that away and stacking every slice at the same height -- which is
        why the map was a floor plan and "everything is at the same height".

        Now the ring is rotated by the FULL orientation before being placed:

            world = Rz(yaw) . Ry(pitch) . Rx(roll) . ring   +   position

        Drive him around while working the right stick and the live map fills in a
        volume instead of a plane. Hold still and sweep -- that's what NOD does, and
        this is now the same maths, just continuous.

        Y is negated to convert the LiDAR's clockwise bearing indexing into a
        right-handed frame (what the raw data calls 'left' is actually Gary's right).
        """
        d = np.asarray(dists, dtype=float)
        idx = np.arange(360)
        good = (d >= self.args.min_mm) & (d <= self.args.max_mm)
        self.dropped += int((~good).sum())
        d, ang = d[good], np.radians(idx[good])

        # The ring, in Gary's body frame. It IS planar -- the LiDAR only spins in
        # one plane. What makes the map 3D is that the PLANE ITSELF moves.
        pr = np.stack([d * np.cos(ang), -d * np.sin(ang), np.zeros_like(d)], axis=1)

        x, y, yaw = pose[0], pose[1], pose[2]
        pitch = pose[3] if len(pose) > 3 else 0.0
        roll  = pose[4] if len(pose) > 4 else 0.0

        cr, sr = math.cos(math.radians(roll)),  math.sin(math.radians(roll))
        cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
        cy, sy = math.cos(math.radians(yaw)),   math.sin(math.radians(yaw))

        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

        pw = pr @ (Rz @ Ry @ Rx).T
        pw[:, 0] += x * 10.0        # cm -> mm
        pw[:, 1] += y * 10.0
        pw[:, 2] += self.args.lidar_z
        return pw, d, idx[good]

    # ---- mover detection ---------------------------------------------------
    def update_movers(self, dists, pose):
        """Live-vs-reference change detection, robot frame.

        Only valid while Gary holds still -- if he moves, the reference is
        garbage (he's legitimately seeing a different world), so we throw it
        away and rebuild. This is the ONLY thing here that can localize a
        mover; CSI can only say that motion happened, not where."""
        if self.ref_pose is None or any(abs(a - b) > POSE_EPS for a, b in zip(pose, self.ref_pose)):
            self.ref, self.ref_stack, self.ref_pose = None, [], pose
            self.movers = []
            self.trail.clear()
            return

        d = np.asarray(dists, dtype=float)
        valid = (d >= self.args.min_mm) & (d <= self.args.max_mm)

        if self.ref is None:
            self.ref_stack.append(np.where(valid, d, np.nan))
            if len(self.ref_stack) >= REF_FRAMES:
                with np.errstate(all="ignore"):
                    self.ref = np.nanmedian(np.stack(self.ref_stack), axis=0)
            return

        with np.errstate(all="ignore"):
            closer = valid & np.isfinite(self.ref) & (self.ref - d > MOVER_DELTA_MM)

        # cluster consecutive bearings (wrapping at 360)
        hits = np.where(closer)[0]
        clusters, cur = [], []
        for i in hits:
            if cur and i == cur[-1] + 1:
                cur.append(i)
            else:
                if len(cur) >= MOVER_MIN_BEARINGS:
                    clusters.append(cur)
                cur = [i]
        if len(cur) >= MOVER_MIN_BEARINGS:
            clusters.append(cur)

        movers = []
        th = math.radians(pose[2])
        c, s = math.cos(th), math.sin(th)
        for cl in clusters:
            ang = np.radians(np.array(cl, dtype=float))
            dd = d[cl]
            xr, yr = float(np.mean(dd * np.cos(ang))), float(np.mean(-dd * np.sin(ang)))
            xw = xr * c - yr * s + pose[0] * 10.0
            yw = xr * s + yr * c + pose[1] * 10.0
            movers.append([round(xw, 1), round(yw, 1), round(self.args.lidar_z, 1),
                           round(float(np.mean(dd)), 0), len(cl)])
        self.movers = movers
        for m in movers:
            self.trail.append(m[:3])

    # ---- line dispatch -----------------------------------------------------
    def handle_line(self, line):
        line = line.strip()
        if not line:
            return

        m = SCAN_RE.match(line)
        if m:
            # pitch/roll are optional -- older firmware doesn't send them, and a
            # missing tilt is not the same as a zero tilt, but for a robot that is
            # usually near level, treating it as zero degrades gracefully to the
            # old flat behaviour rather than throwing.
            pitch = float(m.group(4)) if m.group(4) is not None else 0.0
            roll  = float(m.group(5)) if m.group(5) is not None else 0.0
            rx, ry, rth = float(m.group(1)), float(m.group(2)), float(m.group(3))
            try:
                dists = [int(p) for p in m.group(6).split(",")]
            except ValueError:
                return
            if len(dists) != 360:
                dists = (dists + [0] * 360)[:360]
            # SCAN-MATCH the pose from the lidar itself. His reported odometry is
            # 0,0,0 during EXPLORE (and drifting gyro heading otherwise), which is
            # why the raw map smeared. Keep the reported pitch/roll for the 3D tilt.
            used = self.slam.step((rx, ry, rth), dists)
            sx, sy, sth = self.slam.pose[0] / 10.0, self.slam.pose[1] / 10.0, self.slam.pose[2]
            # The LIVE drive-by map is a FLOOR PLAN -- "maintain fixed locations."
            # Place each ring FLAT (yaw only, at the lidar plane) and do NOT lift
            # it by Gary's body pitch/roll. Bill 2026-09-17 ("projects its pips
            # through the floor"): Gary does not walk level -- measured, he cruises
            # nose-up ~8 deg and up to 25 deg maneuvering (run1/run2), and feeding
            # that body tilt into the 3D lift tipped every ring so distant points
            # sank through the floor (run2: 11% below -300mm, down to -659mm).
            # The 3D ROOM is the nod scan's job (action 14) -- a deliberate
            # STATIONARY sweep -- not the continuous walk-around.
            pose = (sx, sy, sth, 0.0, 0.0)        # flat: body tilt ignored here
            pts, _, _ = self.ring_to_world(dists, pose)
            with self.lock:
                self.pose = (sx, sy, sth)         # marker tracks even when we don't map
                # Map when the pose is TRUSTWORTHY: a confident scan-match lock
                # ("icp") OR a confirmed-stationary frame ("still" = pose
                # unchanged, so the ring sits at a known-good pose -> no smear).
                # The old icp-only gate dropped every "still" frame, and Gary is
                # "still" for most of a slow explore (measured 133/235 frames) --
                # so the map barely grew and Bill saw no pips accumulate. Skip
                # only "odom_fallback"/"gap"/"seed" (guessed poses), the true smear.
                if used in ("icp", "still"):
                    self._add(self.cells_trig if self.in_triggered_scan else self.cells_normal, pts)
                self.rings_seen += 1
                self.last_data_at = time.time()
                self.update_movers(dists, (sx, sy, sth))
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
            # nod slices are robot-frame; place them at the current pose too
            if len(pts):
                with self.lock:
                    px, py, th = self.pose
                    c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
                    xw = pts[:, 0] * c - pts[:, 1] * s + px * 10.0
                    yw = pts[:, 0] * s + pts[:, 1] * c + py * 10.0
                    world = np.stack([xw, yw, pts[:, 2]], axis=1)
                    self._add(self.cells_trig if self.in_triggered_scan else self.cells_normal, world)
                    self.last_data_at = time.time()
            return

        m = RSSI_RE.match(line)
        if m:
            with self.lock:
                self.rssi_t.append(round(time.time() - self.t0, 2))
                self.rssi_v.append(int(m.group(1)))
                self.baseline_v.append(float(m.group(2)))
                self.last_data_at = time.time()
            return

        m = STATUS_RE.search(line)
        if m:
            with self.lock:
                self.phase, self.baseline = m.group(1), float(m.group(2))
                self.samples, self.triggered_count = int(m.group(3)), int(m.group(4))
            return

        m = SCANNER_RE.search(line)
        if m:
            with self.lock:
                self.scanner_on = (m.group(1) == "ON")
            return

        if TRIP_RE.search(line):
            with self.lock:
                self.trips.append(round(time.time() - self.t0, 2))
            return
        if TRIG_START_RE.search(line):
            self.in_triggered_scan = True
            return
        if TRIG_DONE_RE.search(line) or SWEEP_DONE_RE.search(line):
            self.in_triggered_scan = False
            with self.lock:
                self.scans_seen += 1
            return

    def snapshot(self):
        with self.lock:
            pn = list(self.cells_normal.values())
            pt = list(self.cells_trig.values())
            age = (time.time() - self.last_data_at) if self.last_data_at else None
            trail = list(self.trail)
            return {
                "normal": {"x": [p[0] for p in pn], "y": [p[1] for p in pn], "z": [p[2] for p in pn]},
                "trig": {"x": [p[0] for p in pt], "y": [p[1] for p in pt], "z": [p[2] for p in pt]},
                "movers": self.movers,
                "trail": {"x": [p[0] for p in trail], "y": [p[1] for p in trail], "z": [p[2] for p in trail]},
                "pose": self.pose,
                "ref_ready": self.ref is not None,
                "n_normal": len(pn), "n_trig": len(pt),
                "dropped": self.dropped, "scans": self.scans_seen, "rings": self.rings_seen,
                "triggered": self.triggered_count,
                "phase": self.phase, "baseline": self.baseline, "samples": self.samples,
                "rssi_t": list(self.rssi_t), "rssi_v": list(self.rssi_v),
                "baseline_v": list(self.baseline_v), "trips": list(self.trips),
                "age": age, "connected": self.connected, "error": self.error,
                "scanner_on": self.scanner_on,
            }


class Link:
    """Owns the device handle so the HTTP thread can send control chars
    (scanner toggle) without racing the reader thread's reads."""

    def __init__(self):
        self.src = None
        self.wlock = threading.Lock()

    def send(self, data):
        with self.wlock:
            if self.src is not None:
                try:
                    self.src.write(data)
                except Exception:
                    pass


def reader_thread(state, link, args, stop):
    # Auto-reconnecting: Gary brownouts/restarts drop the TCP link, and after a
    # restart his scanner mode defaults OFF. Reconnect and re-arm the scanner so
    # the map keeps flowing across power-cycles without a manual relaunch.
    while not stop.is_set():
        try:
            src = open_source(args.source)
        except Exception as e:
            state.error = "connect failed: %s" % e
            state.connected = False
            time.sleep(2.0)
            continue
        link.src = src
        state.connected = True
        state.error = None
        if not args.no_scanner_request:
            link.send(b"1")
            print("Requested scanner mode ON.")
        print("Connected to %s" % args.source)

        buf = b""
        last_rx = time.time()
        try:
            while not stop.is_set():
                got = False
                try:
                    if getattr(src, "in_waiting", False):
                        chunk = src.readline()
                        if chunk:
                            got = True
                            last_rx = time.time()
                            buf += chunk
                            while b"\n" in buf:
                                line, buf = buf.split(b"\n", 1)
                                try:
                                    state.handle_line(line.decode("utf-8", errors="replace"))
                                except Exception:
                                    pass
                except Exception:
                    break   # socket error -> reconnect
                if not got:
                    time.sleep(0.02)
                    if time.time() - last_rx > 8.0:
                        # no frames for 8s: link dropped or scanner off after a
                        # restart -- cycle the connection to re-arm it
                        print("no frames for 8s -- reconnecting (Gary restart?)")
                        break
        finally:
            try:
                src.close()
            except Exception:
                pass
            state.connected = False
        time.sleep(1.5)


PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>Gary -- live map</title>
<script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
<style>
  body { margin:0; font-family:system-ui,Segoe UI,sans-serif; background:#14161a; color:#e7e9ee; }
  header { padding:8px 14px; border-bottom:1px solid #2a2f38; display:flex; gap:14px; align-items:center; flex-wrap:wrap;}
  h1 { font-size:14px; margin:0; font-weight:600;}
  #status { font-size:12px; color:#9aa3b2; }
  #status.live { color:#4ade80; } #status.stalled { color:#f87171; }
  button { background:#232a35; color:#e7e9ee; border:1px solid #39424f; border-radius:6px;
           padding:5px 10px; font-size:12px; cursor:pointer; }
  button:hover { background:#2c3644; }
  button.on { background:#14532d; border-color:#22c55e; color:#bbf7d0; }
  .hint { font-size:11px; color:#6b7482; }
  .wrap { display:grid; grid-template-columns: 1fr 420px; gap:8px; padding:8px; }
  #cloud,#rssi { height: calc(100vh - 86px); background:#14161a; border:1px solid #2a2f38; border-radius:8px;}
  @media (max-width: 900px){ .wrap{ grid-template-columns:1fr; } #cloud,#rssi{ height:60vh; } }
</style></head>
<body>
<header>
  <h1>Gary &mdash; live map</h1>
  <button id="scan-btn" onclick="toggleScanner()">scanner: ?</button>
  <button onclick="resetMap()">clear map</button>
  <button onclick="resetView()">reset view</button>
  <span id="status">connecting...</span>
  <span class="hint">left-drag orbit &middot; wheel zoom &middot; right-drag pan</span>
</header>
<div class="wrap"><div id="cloud"></div><div id="rssi"></div></div>
<script>
const DARK = {paper_bgcolor:'#14161a', plot_bgcolor:'#14161a', font:{color:'#c8cedb', size:11}};
let inited = false, scannerOn = null;

// THE VIEW WOULD NOT STAY PUT. Three attempts at this, so stop being clever and
// be explicit: REMEMBER WHERE THE USER PUT THE CAMERA, AND PUT IT BACK.
//
// Attempt 1 re-asserted scene.camera every update -- that yanked the view mid-drag.
// Attempt 2 set it only on first plot, but left aspectmode:'data' autoranging, so
//   Plotly re-framed the scene every time the cloud grew.
// Attempt 3 nailed the axes and switched to restyle... but by moving camera INTO
//   the persistent layout, Plotly now had an explicit camera stored in gd.layout and
//   would re-apply it on any internal relayout. I fixed the symptom and re-created
//   the cause.
//
// This does not depend on knowing what Plotly does internally. We listen for the
// user moving the camera, remember exactly where they left it, and restore that
// -- never the initial position. If Plotly snaps back, we snap it straight again.
// FIX 4. The event-based capture lost a race and I should have seen it: Plotly only
// emits plotly_relayout when a drag ENDS. Mid-drag it hasn't fired, so the saved
// camera is stale -- and meanwhile the 700ms poll fires a restyle, Plotly re-applies
// the camera it stored at startup, and you get yanked back WHILE STILL DRAGGING.
// Chasing the event was never going to win that race.
//
// So don't race it. Two things, neither of which depends on an event arriving:
//
//   1. WHILE YOU ARE TOUCHING THE PLOT, WE DO NOT TOUCH IT. Mouse down on the
//      canvas pauses updates; mouse up resumes. You cannot be interrupted by a
//      redraw that never happens.
//   2. Around every update we read the camera out of Plotly's OWN live state and
//      put it straight back. Not a remembered value, not an event payload -- the
//      actual current camera, one line before and one line after.
// FIX 5 -- and the previous four were all chasing a bug I had written myself.
//
// liveCam() used to read gd._fullLayout.scene.camera and call that "where the camera
// IS, right now". IT IS NOT. In Plotly.js that field is the DECLARED camera -- the
// value from the last layout object handed to Plotly, i.e. the startup eye. The live
// camera lives somewhere else entirely (scene._scene.getCamera()). So keepCamera()
// was reading the startup view and writing the startup view back, every 700ms,
// forever. The "safety net" WAS the snap-back. Nothing else was moving the view.
//
// So there is nothing to restore. uirevision:'keep' + restyle-only updates (no layout
// ever passed again after newPlot) means Plotly never has cause to touch the camera.
// Leave it alone and it stays put. The drag guard below is kept only to stop the
// redraw stuttering the framerate while you are orbiting.
let dragging = false;

function watchCamera(id){
  const gd = document.getElementById(id);
  gd.addEventListener('mousedown',  () => { dragging = true;  });
  window.addEventListener('mouseup', () => { dragging = false; });
  gd.addEventListener('wheel', () => { dragging = true; clearTimeout(gd._wt);
                                       gd._wt = setTimeout(() => dragging = false, 400); });
}

function resetView(){
  Plotly.relayout('cloud', {'scene.camera': {eye:{x:1.4,y:1.4,z:1.1}, up:{x:0,y:0,z:1}}});
}

async function toggleScanner(){ await fetch('/scanner?on=' + (scannerOn ? 0 : 1), {method:'POST'}); }
async function resetMap(){ await fetch('/reset', {method:'POST'}); }

function spoke(dir, color, label, L){
  return {type:'scatter3d', mode:'lines+text', x:[0,dir[0]*L], y:[0,dir[1]*L], z:[0,dir[2]*L],
          line:{color:color, width:6}, text:['',label], textposition:'top center',
          textfont:{color:color,size:12}, hoverinfo:'skip', showlegend:false};
}

async function tick(){
  let d;
  try { d = await (await fetch('/data.json')).json(); }
  catch(e){ document.getElementById('status').textContent = 'server gone'; return; }

  scannerOn = d.scanner_on;
  const b = document.getElementById('scan-btn');
  b.textContent = 'scanner: ' + (d.scanner_on === null ? '?' : (d.scanner_on ? 'ON' : 'OFF'));
  b.className = d.scanner_on ? 'on' : '';

  const s = document.getElementById('status');
  const mv = d.movers.length ? ` | ${d.movers.length} MOVER(S)` : '';
  const ref = d.ref_ready ? '' : ' | building reference...';
  if (d.error) { s.textContent = d.error; s.className='stalled'; }
  else if (d.age === null) { s.textContent = 'connected, no data yet -- turn scanner ON'; s.className='stalled'; }
  else if (d.age > 5) { s.textContent = `STALLED -- no data for ${d.age.toFixed(0)}s (scanner off?)`; s.className='stalled'; }
  else { s.textContent = `live -- ${d.n_normal + d.n_trig} pts | ${d.rings} rings, ${d.scans} nod-scans | pose ${d.pose[0].toFixed(0)},${d.pose[1].toFixed(0)}cm @${d.pose[2].toFixed(0)}°${ref}${mv}`;
         s.className='live'; }

  const L = 400;
  const cloud = [
    {type:'scatter3d', mode:'markers', name:'map',
     x:d.normal.x, y:d.normal.y, z:d.normal.z,
     marker:{size:1.4, color:d.normal.z, colorscale:'Viridis', opacity:0.8}},
    {type:'scatter3d', mode:'markers', name:'CSI-triggered',
     x:d.trig.x, y:d.trig.y, z:d.trig.z,
     marker:{size:2.4, color:'orangered', opacity:0.95}},
    {type:'scatter3d', mode:'markers', name:'mover trail',
     x:d.trail.x, y:d.trail.y, z:d.trail.z,
     marker:{size:3, color:'#fbbf24', opacity:0.5}},
    {type:'scatter3d', mode:'markers', name:'MOVER',
     x:d.movers.map(m=>m[0]), y:d.movers.map(m=>m[1]), z:d.movers.map(m=>m[2]),
     marker:{size:10, color:'#ef4444', symbol:'x'}},
    {type:'scatter3d', mode:'markers+text', x:[d.pose[0]*10], y:[d.pose[1]*10], z:[0],
     marker:{size:6, color:'red', symbol:'diamond'}, text:['GARY'],
     textposition:'bottom center', textfont:{color:'red',size:12}, showlegend:false},
    spoke([1,0,0],'red','FWD',L), spoke([0,1,0],'green','LEFT',L), spoke([0,0,1],'blue','UP',L)
  ];

  // THE VIEW KEPT SNAPPING BACK. Setting scene.camera only on the first plot was
  // necessary but NOT sufficient, and here is the part I missed: aspectmode:'data'
  // with autoranging axes makes Plotly RECOMPUTE the whole scene bounding box on
  // every single update. The cloud grows by a few hundred points a second, the box
  // grows with it, and Plotly obediently re-frames the scene -- dragging your view
  // out from under you. uirevision preserves the camera, but it cannot preserve a
  // camera aimed at a target that keeps moving.
  //
  // So: nail the axes to a fixed world volume (Gary's LiDAR tops out at 8m anyway)
  // and NEVER pass a layout again after the first plot. Below, updates go through
  // Plotly.restyle, which touches data only and cannot disturb the camera.
  const R = 8000, ZR = 2000;
  const layout3d = Object.assign({}, DARK, {
    margin:{l:0,r:0,t:0,b:0}, uirevision:'keep',
    scene:{ xaxis:{title:'X fwd (mm)',color:'#8b93a1',gridcolor:'#242a33',
                   range:[-R,R], autorange:false},
            yaxis:{title:'Y left (mm)',color:'#8b93a1',gridcolor:'#242a33',
                   range:[-R,R], autorange:false},
            zaxis:{title:'Z up (mm)',color:'#8b93a1',gridcolor:'#242a33',
                   range:[-ZR,ZR], autorange:false},
            aspectmode:'data', bgcolor:'#14161a', dragmode:'turntable',
            camera:{eye:{x:1.4,y:1.4,z:1.1}, up:{x:0,y:0,z:1}},
            uirevision:'keep' },
    legend:{x:0,y:1, bgcolor:'rgba(0,0,0,0)'}
  });

  const rssi = [
    {x:d.rssi_t, y:d.rssi_v, mode:'lines', name:'RSSI', line:{color:'#8b93a1', width:1}},
    {x:d.rssi_t, y:d.baseline_v, mode:'lines', name:'baseline', line:{color:'#60a5fa', width:2}}
  ];
  const layoutR = Object.assign({}, DARK, {
    margin:{l:50,r:10,t:34,b:40}, uirevision:'keep',
    title:{text:`phase=${d.phase}  baseline=${d.baseline}  triggered=${d.triggered}`, font:{size:11}},
    xaxis:{title:'seconds', gridcolor:'#242a33'}, yaxis:{title:'RSSI (dBm)', gridcolor:'#242a33'},
    shapes: d.trips.map(t => ({type:'line', x0:t, x1:t, yref:'paper', y0:0, y1:1,
                               line:{color:'#ef4444', width:1, dash:'dash'}})),
    legend:{orientation:'h', y:-0.2}
  });

  // HANDS OFF WHILE THE USER IS ORIENTING THE VIEW. The map is still accumulating
  // in the background -- we just don't redraw it until they let go.
  if (dragging && inited) return;

  const cfg = {scrollZoom:true, displaylogo:false, responsive:true};
  if (!inited){
    Plotly.newPlot('cloud', cloud, layout3d, cfg);
    Plotly.newPlot('rssi', rssi, layoutR, cfg);
    watchCamera('cloud');
    inited = true;
  } else {
    // DATA ONLY. Never hand Plotly a layout again -- not via react, not via relayout,
    // not "just to put the camera back". The 3D panel's layout is written once, at
    // newPlot, and never spoken of again. That is the whole fix.
    Plotly.restyle('cloud', {
      x: cloud.map(t => t.x), y: cloud.map(t => t.y), z: cloud.map(t => t.z)
    });
    Plotly.restyle('rssi', {x: rssi.map(t => t.x), y: rssi.map(t => t.y)});
    // The RSSI title and TRIP markers do live in the layout, but that panel is 2D
    // and has no camera to lose, so relayout is harmless there.
    Plotly.relayout('rssi', {'title.text': layoutR.title.text, shapes: layoutR.shapes});
  }
}
tick(); setInterval(tick, POLL_MS);
</script></body></html>
"""


def make_handler(state, link, poll_ms):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, body, ctype):
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            if self.path.startswith("/scanner"):
                on = "on=1" in self.path
                link.send(b"1" if on else b"0")
                with state.lock:
                    state.scanner_on = on   # optimistic; Gary's echo confirms
                self._send(b'{"ok":true}', "application/json")
            elif self.path.startswith("/reset"):
                with state.lock:
                    state.cells_normal.clear()
                    state.cells_trig.clear()
                    state.ref, state.ref_stack, state.ref_pose = None, [], None
                    state.trail.clear()
                    state.movers = []
                    state.slam = Slam(mode="scanmatch")   # restart the map from here
                    state.pose = (0.0, 0.0, 0.0)
                self._send(b'{"ok":true}', "application/json")
            else:
                self.send_response(404)
                self.end_headers()

        def do_GET(self):
            if self.path.startswith("/data.json"):
                self._send(json.dumps(state.snapshot()).encode(), "application/json")
            else:
                self._send(PAGE.replace("POLL_MS", str(poll_ms)).encode(), "text/html; charset=utf-8")

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source", default="tcp:192.168.1.156:3333")
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--poll-ms", type=int, default=700)
    # Max-detail defaults. NOTE: nod_pointcloud hard-clamps at 4000mm on import;
    # we override its module constants below, which is what actually lifts it.
    ap.add_argument("--min-mm", type=float, default=100.0)
    ap.add_argument("--max-mm", type=float, default=8000.0)
    ap.add_argument("--voxel-mm", type=float, default=15.0)
    ap.add_argument("--max-points", type=int, default=300000)
    ap.add_argument("--lidar-z", type=float, default=0.0, help="height of the LiDAR plane in mm")
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--no-scanner-request", action="store_true")
    args = ap.parse_args()
    if args.voxel_mm <= 0:
        args.voxel_mm = 1e-6

    # This is the clamp that was quietly throwing away everything past 4m.
    npc.MIN_RANGE_MM = args.min_mm
    npc.MAX_RANGE_MM = args.max_mm

    state = State(args)
    link = Link()
    stop = threading.Event()
    t = threading.Thread(target=reader_thread, args=(state, link, args, stop), daemon=True)
    t.start()

    srv = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(state, link, args.poll_ms))
    url = "http://localhost:%d" % args.port
    print("\n  Gary's live map:  %s" % url)
    print("  (phone on same LAN: http://<this-PC-ip>:%d )\n  Ctrl+C to stop.\n" % args.port)
    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()

    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down ...")
    finally:
        stop.set()
        srv.shutdown()
        t.join(timeout=2)


if __name__ == "__main__":
    main()
