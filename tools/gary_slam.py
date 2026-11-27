#!/usr/bin/env python3
"""gary_slam.py -- scan-matching SLAM: the drift-corrected map the odom fitter
itself called "the real fix" (gary_odom_fit.py: "scan-matching the map itself,
which is a bigger job").

THE PROBLEM (Bill, 2026-09-17, watching the live viewer): "it isn't maintaining
fixed locations, it's not really mapping." The web dashboard plots each lidar
ring at Gary's REPORTED pose. Position comes from the legs (good, ~32mm RMS) but
HEADING comes from the gyro, whose yaw scale was never calibrated and which
drifts -- so as he turns, the whole cloud rotates off and smears. Every power-off
also snaps his pose back to 0,0, overlaying old and new.

THE FIX: take HEADING from lidar scan-matching instead of the gyro. Consecutive
rings overlap hugely, so ICP recovers the TRUE rotation between them with no
drift and no calibration. We keep the good leg odometry for translation and only
replace the bad heading -- "legs for position, SCAN-MATCH for heading." Where the
scene is too featureless for ICP to lock (open room, bare corridor) we fall back
to raw odometry for that step. Reuses the proven ring-correlation + point-to-line
ICP from gary_odom_fit.py; nothing new to trust in the hard math.

    python gary_slam.py --selfcheck                     # prove it on a synthetic room
    python gary_slam.py --source tcp:gary.local:3333    # live, top-down map window
    python gary_slam.py --source file:run.scan          # replay a recorded SCAN log

Modes:  --mode fuse  (default: odom translation + scan-match heading)
        --mode scanmatch (full ICP motion; better in feature-rich rooms, worse in
                          long corridors where ICP translation slides)
"""
import argparse
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gary_odom_fit import (ring_points, yaw_by_correlation, icp, to_midpoint_frame,
                           odom_body_delta, wrap180, Reader, SCAN_RE,
                           MIN_MM, MAX_MM, MAX_RESIDUAL_MM, MIN_CORR_QUALITY,
                           MIN_MOVE_MM, MIN_TURN_DEG, MAX_MOVE_MM, MAX_TURN_DEG)


def integrate(pose, dx_mm, dy_mm, dth_deg):
    """Midpoint-rule world integration (matches the firmware). pose=(x_mm,y_mm,th_deg).
    Body frame: x forward, y left."""
    x, y, th = pose
    thm = math.radians(th + dth_deg / 2.0)
    c, s = math.cos(thm), math.sin(thm)
    return (x + dx_mm * c - dy_mm * s, y + dx_mm * s + dy_mm * c, th + dth_deg)


def ring_world(pose, dists):
    """Current ring's points transformed into the world frame (mm)."""
    x, y, th = pose
    pts = ring_points(dists)
    c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
    return np.stack([x + pts[:, 0] * c - pts[:, 1] * s,
                     y + pts[:, 0] * s + pts[:, 1] * c], axis=1)


def voxel(pts, cell=30.0):
    if len(pts) == 0:
        return pts
    k = np.round(pts / cell).astype(np.int64)
    _, idx = np.unique(k, axis=0, return_index=True)
    return pts[idx]


class Slam:
    """Scan-match heading, leg-odometry translation, raw-odometry fallback."""

    def __init__(self, mode="fuse"):
        self.mode = mode
        self.pose = (0.0, 0.0, 0.0)        # corrected/fused
        self.pose_odom = (0.0, 0.0, 0.0)   # raw odometry, for side-by-side
        self.prev = None
        self.map_pts = np.zeros((0, 2))
        self.traj = [(0.0, 0.0)]
        self.traj_odom = [(0.0, 0.0)]
        self.last_step_t = 0.0
        self.stats = {"icp": 0, "odom_fallback": 0, "still": 0, "gap": 0}

    GAP_RESEED_S = 1.0   # >1s between frames (nod scan, stall) -> don't match across it

    def step(self, odom_pose, dists):
        nowt = time.time()
        if self.prev is None:
            self.prev = (odom_pose, dists); self.last_step_t = nowt
            return "seed"
        if self.last_step_t and (nowt - self.last_step_t) > self.GAP_RESEED_S:
            # long gap: the previous ring is stale (e.g. a 40s nod scan). Matching
            # across it produces a wild jump -> re-seed from here instead.
            self.prev = (odom_pose, dists); self.last_step_t = nowt
            self.traj.append((self.pose[0], self.pose[1]))
            self.stats["gap"] += 1
            return "gap"
        self.last_step_t = nowt
        p0, d0 = self.prev
        p1, d1 = odom_pose, dists
        o_fwd, o_lat, o_dth = odom_body_delta(p0, p1)   # mm, mm, deg (midpoint frame)

        self.pose_odom = integrate(self.pose_odom, o_fwd, o_lat, o_dth)

        dx = dy = dth = 0.0
        used = "still"
        # Attempt scan-match EVERY frame -- odometry is frozen at 0,0,0 during
        # autonomous EXPLORE, so the lidar is the only motion source there.
        icp_ok = False
        yaw_c, q = yaw_by_correlation(d0, d1)
        if yaw_c is not None and q >= MIN_CORR_QUALITY:
            yaw_t, t_t, rms = icp(ring_points(d1), ring_points(d0), init_yaw_deg=yaw_c)
            dxm, dym = to_midpoint_frame(t_t, yaw_t)
            small = (math.hypot(dxm, dym) <= MAX_MOVE_MM) and (abs(yaw_t) <= MAX_TURN_DEG)
            if np.isfinite(rms) and rms <= MAX_RESIDUAL_MM and small:
                icp_ok = True
                moved = (math.hypot(dxm, dym) > MIN_MOVE_MM) or (abs(yaw_t) > MIN_TURN_DEG)
                if self.mode == "scanmatch":
                    dx, dy, dth = dxm, dym, yaw_t
                else:  # fuse: leg translation (needs live odom) + scan-match heading
                    dx, dy, dth = o_fwd, o_lat, yaw_t
                if moved:
                    used = "icp"; self.stats["icp"] += 1
        if not icp_ok:
            # ICP couldn't lock -- lean on odometry if IT says he moved
            if abs(o_fwd) > MIN_MOVE_MM or abs(o_dth) > MIN_TURN_DEG:
                dx, dy, dth = o_fwd, o_lat, o_dth
                used = "odom_fallback"; self.stats["odom_fallback"] += 1
            else:
                self.stats["still"] += 1

        self.pose = integrate(self.pose, dx, dy, dth)
        self.traj.append((self.pose[0], self.pose[1]))
        self.traj_odom.append((self.pose_odom[0], self.pose_odom[1]))
        if used == "icp":     # only map when the pose is scan-match CONFIDENT (no smear)
            wp = ring_world(self.pose, d1)
            self.map_pts = voxel(np.vstack([self.map_pts, wp])) if len(self.map_pts) else voxel(wp)
        self.prev = (odom_pose, dists)
        return used


# ============================ SELF-CHECK ====================================
def _raycast_room(pose, W, H, n=360):
    """Ground-truth 360 ranges (mm) for a robot in an axis-aligned room [0,W]x[0,H].
    Bearing i's body direction is (cos i, -sin i) to match ring_points()."""
    x, y, th = pose
    thr = math.radians(th)
    ct, st = math.cos(thr), math.sin(thr)
    out = []
    for i in range(n):
        a = math.radians(i)
        # body ray dir (cos a, -sin a) -> world
        bx, by = math.cos(a), -math.sin(a)
        wx = bx * ct - by * st
        wy = bx * st + by * ct
        best = MAX_MM
        for (num, den) in ((0 - x, wx), (W - x, wx), (0 - y, wy), (H - y, wy)):
            if abs(den) < 1e-9:
                continue
            tt = num / den
            if tt <= 1.0:
                continue
            px, py = x + tt * wx, y + tt * wy
            if -1.0 <= px <= W + 1.0 and -1.0 <= py <= H + 1.0:
                best = min(best, tt)
        out.append(int(round(min(best, MAX_MM))) if best < MAX_MM else 0)
    return out


def selfcheck():
    """Synthetic room + known trajectory. Feed TRUE rings but odometry whose HEADING
    is corrupted by a 15% gyro-yaw error (the real bug). SLAM must recover the true
    path far better than raw odometry does."""
    print("\n  self-check: synthetic 8x8 m room, gyro yaw error 1.15x\n")
    W, H = 8000.0, 8000.0
    # a tight central square loop (stays well inside the room so every ring sees
    # all four walls): forward in 80mm steps, turn in 6deg steps, x4 corners.
    seg = [(80.0, 0.0, 0.0)] * 15 + [(0.0, 0.0, 6.0)] * 15
    motions = seg * 4
    YAW_ERR = 1.15

    true_pose = (3400.0, 3400.0, 0.0)
    odom_pose = (3400.0, 3400.0, 0.0)
    sl = Slam(mode="fuse")
    sl.pose = true_pose            # both start at the same known origin
    sl.pose_odom = true_pose
    sl.prev = (odom_pose, _raycast_room(true_pose, W, H))

    truth = [true_pose]
    for (fwd, lat, dth) in motions:
        true_pose = integrate(true_pose, fwd, lat, dth)
        odom_pose = integrate(odom_pose, fwd, lat, dth * YAW_ERR)   # legs true, gyro wrong
        truth.append(true_pose)
        sl.step(odom_pose, _raycast_room(true_pose, W, H))

    tx, ty, tth = true_pose
    sx, sy, sth = sl.pose
    ox, oy, oth = sl.pose_odom
    err_odom = math.hypot(ox - tx, oy - ty)
    err_slam = math.hypot(sx - tx, sy - ty)
    hd_odom = abs(wrap180(oth - tth))
    hd_slam = abs(wrap180(sth - tth))
    print(f"    final position error   raw-odom {err_odom:7.1f} mm   SLAM {err_slam:7.1f} mm")
    print(f"    final heading error    raw-odom {hd_odom:7.2f} deg  SLAM {hd_slam:7.2f} deg")
    print(f"    icp-locked steps: {sl.stats['icp']}   odom-fallback: {sl.stats['odom_fallback']}")
    ok = (err_slam < 0.30 * err_odom + 1e-9) and (hd_slam < 0.30 * hd_odom + 1e-9) and sl.stats["icp"] > 20
    print(f"\n  {'PASS -- scan-matching cut the drift hard' if ok else 'FAIL -- do not trust this'}\n")
    return 0 if ok else 1


# ============================ LIVE / REPLAY =================================
def file_frames(path):
    """Replay SCAN: lines from a recorded log file."""
    with open(path, "r", errors="replace") as f:
        for line in f:
            m = SCAN_RE.match(line.strip())
            if not m:
                continue
            try:
                dists = [int(x) for x in m.group(6).split(",")]
            except ValueError:
                continue
            if len(dists) == 360:
                yield (float(m.group(1)), float(m.group(2)), float(m.group(3))), dists


def run_live(args):
    sl = Slam(mode=args.mode)
    plt = None
    if not args.no_plot:
        try:
            import matplotlib.pyplot as plt
            plt.ion()
            fig, ax = plt.subplots(figsize=(8, 6))
        except Exception as e:
            print("matplotlib unavailable (%s) -- running headless, saving PNG only" % e)
            plt = None

    if args.source.startswith("file:"):
        gen = file_frames(args.source[5:])
        src = None
    else:
        from gary_lidar_io import open_source, set_scanner_mode
        src = open_source(args.source)
        set_scanner_mode(src, True)
        time.sleep(0.5)
        reader = Reader(src)

        def _live():
            while True:
                any_frame = False
                for pose, dists in reader.frames():
                    any_frame = True
                    yield pose, dists
                if not any_frame:
                    time.sleep(0.01)
        gen = _live()

    print("gary_slam: mode=%s  source=%s" % (args.mode, args.source))
    last_draw = 0.0
    try:
        for pose, dists in gen:
            sl.step(pose, dists)
            now = time.time()
            if plt and now - last_draw > 0.3:
                last_draw = now
                ax.clear()
                if len(sl.map_pts):
                    mp = sl.map_pts / 1000.0
                    ax.scatter(mp[:, 0], mp[:, 1], s=1, c="#3a6", alpha=0.5)
                tj = np.array(sl.traj) / 1000.0
                to = np.array(sl.traj_odom) / 1000.0
                ax.plot(to[:, 0], to[:, 1], "-", c="#c33", lw=1, alpha=0.6, label="raw odom")
                ax.plot(tj[:, 0], tj[:, 1], "-", c="#06c", lw=2, label="scan-matched")
                ax.plot(sl.pose[0] / 1000.0, sl.pose[1] / 1000.0, "o", c="#06c")
                ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(loc="upper right")
                ax.set_title("Gary SLAM  |  icp %d  fallback %d  |  pts %d"
                             % (sl.stats["icp"], sl.stats["odom_fallback"], len(sl.map_pts)))
                plt.pause(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        if src is not None:
            from gary_lidar_io import set_scanner_mode
            try: set_scanner_mode(src, False); src.close()
            except Exception: pass
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gary_slam_map.png")
        try:
            if plt:
                fig.savefig(out, dpi=130); print("map -> %s" % out)
        except Exception:
            pass
        print("final: pose=(%.0f,%.0f,%.1f)  icp=%d fallback=%d still=%d"
              % (*sl.pose, sl.stats["icp"], sl.stats["odom_fallback"], sl.stats["still"]))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selfcheck", action="store_true")
    ap.add_argument("--source", default="tcp:gary.local:3333")
    ap.add_argument("--mode", choices=["fuse", "scanmatch"], default="fuse")
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args()
    if a.selfcheck:
        return selfcheck()
    return run_live(a)


if __name__ == "__main__":
    raise SystemExit(main())
