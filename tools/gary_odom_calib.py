#!/usr/bin/env python3
"""
gary_odom_calib.py -- calibrate Gary's dead-reckoning odometry against his own LiDAR.

WHY
odom_x / odom_y / odom_theta_deg are integrated from commanded velocities scaled
by two constants in GARY_v3_autoscan.ino:

    static float ODOM_VY_SCALE    = 1.0f;   // cm/s per unit of send_vy
    static float ODOM_OMEGA_SCALE = 1.0f;   // deg/s per unit of send_omega

Both are placeholders. Until they're right, every SCAN: pose is wrong, so the
world-frame map smears and drifts. The usual fix is "drive a measured metre with
a tape measure and fiddle." We can do better: the LiDAR already measures the room
to a few mm, so it can tell us how far Gary ACTUALLY moved. We compare that to
what odometry CLAIMED and the ratio is the correction.

    true_translation / claimed_translation  ->  new ODOM_VY_SCALE
    true_rotation    / claimed_rotation     ->  new ODOM_OMEGA_SCALE

HOW THE GROUND TRUTH WORKS
  * Rotation: cross-correlate two 360-point rings. Turning in place shifts the
    whole range profile by exactly the yaw angle, so the shift that maximizes
    correlation IS the true yaw, to ~1 degree. Very robust -- no room geometry
    assumptions.
  * Translation: 2D ICP (iterative closest point) between the start ring and the
    end ring. Rigid-fits one to the other and reports the (dx, dy) that aligns
    them. Needs some structure in view (a corner, furniture -- a bare corridor of
    two parallel walls is degenerate along its length; the script warns you).

USAGE
    python gary_odom_calib.py --source tcp:192.168.1.156:3333

It walks you through two runs. You drive with the PS5 pad -- odometry only
integrates during controller-driven motion, which is exactly the case we want to
calibrate:

    1. STRAIGHT run: press ENTER, drive Gary straight forward ~1-2m, press ENTER.
    2. TURN run:     press ENTER, spin him in place ~90-180 deg, press ENTER.

Then it prints the two constants to paste into the .ino.

CAVEATS, HONESTLY
  * Hexapod gait slip is load- and surface-dependent. A scale fitted on carpet
    will be off on tile. Calibrate on the surface you'll actually map.
  * This fits a single linear scale. Real gait error isn't perfectly linear with
    commanded velocity, so expect good-not-perfect; it turns "unusable" into
    "drifts slowly," not into SLAM.
  * ICP has no loop closure and can converge to a local minimum in a featureless
    space. The script reports its fit residual so you can tell when to distrust it.
"""

import argparse
import math
import re
import sys
import time

import numpy as np

from gary_lidar_io import open_source, set_scanner_mode

SCAN_RE = re.compile(r"^SCAN:(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?);(.+)$")

MIN_MM, MAX_MM = 120.0, 8000.0


def ring_points(dists):
    """360 ranges -> Nx2 array of (x,y) in robot frame, mm. x fwd, y left."""
    d = np.asarray(dists, dtype=float)
    idx = np.arange(360)
    good = (d >= MIN_MM) & (d <= MAX_MM)
    d, ang = d[good], np.radians(idx[good])
    return np.stack([d * np.cos(ang), -d * np.sin(ang)], axis=1)


def yaw_by_correlation(d0, d1):
    """True yaw change (deg) from two rings, by circular cross-correlation.

    A pure rotation in place shifts the range-vs-bearing profile by exactly the
    yaw. Missing returns are filled with the profile mean so dropouts don't
    dominate the correlation."""
    a = np.asarray(d0, dtype=float).copy()
    b = np.asarray(d1, dtype=float).copy()
    for v in (a, b):
        bad = (v < MIN_MM) | (v > MAX_MM)
        v[bad] = np.nan
    if np.all(np.isnan(a)) or np.all(np.isnan(b)):
        return None, 0.0
    a = np.where(np.isnan(a), np.nanmean(a), a)
    b = np.where(np.isnan(b), np.nanmean(b), b)
    a -= a.mean()
    b -= b.mean()
    # circular correlation via FFT; argmax = shift in degrees
    corr = np.real(np.fft.ifft(np.fft.fft(a) * np.conj(np.fft.fft(b))))
    shift = int(np.argmax(corr))
    if shift > 180:
        shift -= 360
    denom = (np.linalg.norm(a) * np.linalg.norm(b))
    quality = float(corr.max() / denom) if denom else 0.0
    return float(shift), quality


def icp(src, dst, iters=40, tol=1e-4):
    """Minimal 2D point-to-point ICP. Returns (R, t, rms, ok)."""
    if len(src) < 20 or len(dst) < 20:
        return np.eye(2), np.zeros(2), float("inf"), False
    R = np.eye(2)
    t = np.zeros(2)
    prev = None
    cur = src.copy()
    for _ in range(iters):
        # nearest neighbours (brute force; 360 pts, cheap)
        d2 = ((cur[:, None, :] - dst[None, :, :]) ** 2).sum(axis=2)
        j = d2.argmin(axis=1)
        rms = float(np.sqrt(d2[np.arange(len(cur)), j].mean()))
        # reject the worst 20% as outliers (movers, dropouts, new geometry)
        keep = d2[np.arange(len(cur)), j] <= np.quantile(d2[np.arange(len(cur)), j], 0.8)
        P, Q = cur[keep], dst[j[keep]]
        pc, qc = P.mean(axis=0), Q.mean(axis=0)
        H = (P - pc).T @ (Q - qc)
        U, _, Vt = np.linalg.svd(H)
        Ri = Vt.T @ U.T
        if np.linalg.det(Ri) < 0:
            Vt[-1, :] *= -1
            Ri = Vt.T @ U.T
        ti = qc - Ri @ pc
        cur = (Ri @ cur.T).T + ti
        R = Ri @ R
        t = Ri @ t + ti
        if prev is not None and abs(prev - rms) < tol:
            break
        prev = rms
    return R, t, rms, True


def read_scan(src, timeout=6.0):
    """Block until a FRESH SCAN: frame arrives. Returns (pose, dists).

    TWO bugs live here, both of which produced a bogus "odometry barely moved":

    1. STALE BACKLOG. Gary streams ~5 SCAN frames/sec into the TCP socket and
       nobody reads them while you're busy driving. The kernel receive buffer
       builds a backlog dozens of frames deep, so naively reading one line hands
       back the OLDEST queued frame -- captured seconds ago, with the pose Gary
       had back then. First run: START and END both returned the same stale
       frame (pose 0,0,0), so ICP compared a ring against itself. 0mm, 0deg,
       correlation quality 1.00.

    2. THE STREAM IS NEVER QUIET. The obvious fix -- "drain until the wire goes
       silent, then take the newest" -- never terminates early, because at 5Hz
       something always arrives within a few hundred ms. It just blocks for the
       whole timeout and returns whatever was newest at THAT moment, which can
       be a frame from after you already drove.

    So: drain everything currently queued and throw it away, then block for the
    NEXT frame. That frame was necessarily produced after the drain, i.e. it
    reflects where Gary is right now."""
    # --- phase 1: drain the backlog (discard) -------------------------------
    while getattr(src, "in_waiting", False):
        if not src.readline():
            break

    # --- phase 2: the next complete frame is fresh by construction ----------
    buf = b""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if getattr(src, "in_waiting", False):
            chunk = src.readline()
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                m = SCAN_RE.match(line.decode("utf-8", errors="replace").strip())
                if m:
                    try:
                        dists = [int(x) for x in m.group(4).split(",")]
                    except ValueError:
                        continue
                    if len(dists) == 360:
                        pose = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
                        return pose, dists
        else:
            time.sleep(0.005)

    return None, None


def run_leg(src, name, prompt_go, prompt_stop):
    print("\n=== %s ===" % name)
    input("  Put Gary where you want him, then press ENTER to capture the START ring...")
    p0, d0 = read_scan(src)   # drains backlog -> Gary's pose NOW
    if d0 is None:
        print("  No SCAN: frame arrived. Is scanner mode on? Is Gary streaming?", file=sys.stderr)
        sys.exit(1)
    print("  START pose (odom): x=%.1fcm y=%.1fcm th=%.1fdeg" % p0)
    print("  %s" % prompt_go)
    input("  ...then press ENTER to capture the END ring: ")
    p1, d1 = read_scan(src)   # drains backlog -> Gary's pose NOW
    if d1 is None:
        print("  No SCAN: frame arrived at the end.", file=sys.stderr)
        sys.exit(1)
    print("  END   pose (odom): x=%.1fcm y=%.1fcm th=%.1fdeg" % p1)
    if d0 == d1:
        print("  ! START and END rings are byte-identical -- no fresh data arrived.")
        print("    Gary is probably not streaming (scanner mode off?), or he did not move.")
    return (p0, d0), (p1, d1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="tcp:192.168.1.156:3333")
    ap.add_argument("--vy-scale", type=float, default=1.0, help="ODOM_VY_SCALE currently in the .ino")
    ap.add_argument("--omega-scale", type=float, default=1.0, help="ODOM_OMEGA_SCALE currently in the .ino")
    args = ap.parse_args()

    print("Connecting to %s ..." % args.source)
    src = open_source(args.source)
    set_scanner_mode(src, True)
    time.sleep(0.5)
    print("Connected, scanner mode requested ON.")

    # ---------- LEG 1: straight ----------
    (p0, d0), (p1, d1) = run_leg(
        src, "LEG 1 of 2: STRAIGHT",
        "Now drive Gary STRAIGHT FORWARD about 1-2 m with the pad (no turning)...",
        "")
    A, B = ring_points(d0), ring_points(d1)
    R, t, rms, ok = icp(B, A)     # fit END ring back onto START ring
    true_dist_mm = float(np.linalg.norm(t))
    yaw_leg1, q1 = yaw_by_correlation(d0, d1)

    claimed_mm = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) * 10.0
    print("\n  LiDAR (ICP) says he actually moved : %.0f mm   (fit residual %.0f mm)" % (true_dist_mm, rms))
    print("  Odometry claimed he moved          : %.0f mm" % claimed_mm)
    if yaw_leg1 is not None and abs(yaw_leg1) > 8:
        print("  ! He rotated %.0f deg during the 'straight' leg -- redo it straighter." % yaw_leg1)
    if rms > 120:
        print("  ! High ICP residual -- featureless space or he moved too far. Distrust this.")

    if claimed_mm < 50:
        print("  ! Odometry barely moved. Either the scales are ~0, or odom isn't")
        print("    integrating (it only runs in CONTROLLER mode -- drive with the pad).")
        new_vy = None
    else:
        new_vy = args.vy_scale * (true_dist_mm / claimed_mm)

    # ---------- LEG 2: turn ----------
    (q0, e0), (q1p, e1) = run_leg(
        src, "LEG 2 of 2: TURN IN PLACE",
        "Now SPIN Gary in place about 90-180 deg (try not to translate)...",
        "")
    true_yaw, quality = yaw_by_correlation(e0, e1)
    claimed_yaw = q1p[2] - q0[2]
    while claimed_yaw > 180:
        claimed_yaw -= 360
    while claimed_yaw < -180:
        claimed_yaw += 360

    print("\n  LiDAR (ring correlation) says he turned : %+.0f deg   (corr quality %.2f)" % (true_yaw, quality))
    print("  Odometry claimed he turned              : %+.0f deg" % claimed_yaw)
    if quality < 0.3:
        print("  ! Weak correlation -- rings don't match well. Distrust this.")

    if abs(claimed_yaw) < 5:
        print("  ! Odometry barely turned -- same caveat as above.")
        new_omega = None
    else:
        new_omega = args.omega_scale * (true_yaw / claimed_yaw)

    # ---------- result ----------
    print("\n" + "=" * 68)
    print("PASTE INTO GARY_v3_autoscan.ino (around line 2002):")
    print("=" * 68)
    if new_vy:
        print("  static float ODOM_VY_SCALE    = %.4ff;   // was %.4f (LiDAR-calibrated)" % (new_vy, args.vy_scale))
    else:
        print("  ODOM_VY_SCALE    : NOT DETERMINED -- see warnings above")
    if new_omega:
        print("  static float ODOM_OMEGA_SCALE = %.4ff;   // was %.4f (LiDAR-calibrated)" % (new_omega, args.omega_scale))
    else:
        print("  ODOM_OMEGA_SCALE : NOT DETERMINED -- see warnings above")
    print("=" * 68)
    print("\nReflash, then re-run this to VERIFY: both ratios should come back ~1.00.")
    print("If they don't, the error isn't a constant scale factor -- most likely gait")
    print("slip that varies with speed. Calibrate at the speed you actually map at.\n")

    set_scanner_mode(src, False)
    src.close()


if __name__ == "__main__":
    main()
