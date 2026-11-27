#!/usr/bin/env python3
"""
gary_lidar_diag.py -- is Gary's SCAN ring a SNAPSHOT, or a slow smear?

WHY THIS EXISTS
The odometry regression came back all zeros: every coefficient ~0.000, every
R^2 ~0.00. That is not a noisy fit, it is a NULL fit -- the LiDAR reported that
Gary didn't move, regardless of what odometry claimed. The only way that happens
is if the two rings being compared are nearly IDENTICAL.

Suspicion, from Gary's own LIDAR diag lines:

    pkts=16544 @ t=747s   ->   pkts=21103 @ t=995s
    = 4559 packets in 248 s = ~18 pkt/s = roughly 220 points/sec

A 360-point ring at 220 pts/s takes ~1.6 SECONDS to fully repaint. That LiDAR
should be delivering 2000-4000 pts/s. If this is right then:

  * gary_lidar_distance_at_angle() is a persistent buffer, and each SCAN: frame
    is a MOSAIC stitched from ~1.6s of Gary walking -- not an instant snapshot.
  * consecutive SCAN frames (200ms apart) share ~85% of identical stale numbers,
    so frame-to-frame ICP is anchored at zero motion. Exactly what we saw.
  * every map he has ever built has been smeared by this, and every "movers"
    detection has been comparing against partially-stale bearings.

This script measures it directly instead of guessing. STAND GARY STILL, don't
touch the pad, and run it.

WHAT IT REPORTS
  1. points/sec actually arriving, from the LIDAR diag line's packet counter
  2. per-frame CHURN: what fraction of the 360 bearings changed since last frame
  3. FULL-REFRESH TIME: how long until every bearing has been rewritten at least
     once. This is the number that matters. If it's ~200ms, the ring is a real
     snapshot and my theory is wrong. If it's >1s, the ring is a smear and no
     amount of ICP cleverness will save the calibration until it's fixed.

    python gary_lidar_diag.py --source tcp:192.168.1.156:3333
"""

import argparse
import re
import sys
import time

import numpy as np

from gary_lidar_io import open_source, set_scanner_mode

SCAN_RE = re.compile(r"^SCAN:(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)(?:,(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?))?;(.+)$")
LIDAR_RE = re.compile(r">>> LIDAR \| ready=\d+ bytes=(\d+) pkts=(\d+) bad=(\d+)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="tcp:192.168.1.156:3333")
    ap.add_argument("--seconds", type=float, default=25.0)
    args = ap.parse_args()

    print("Connecting to %s ..." % args.source)
    src = open_source(args.source)
    set_scanner_mode(src, True)
    time.sleep(0.5)
    print("Connected.\n")
    print("*** STAND GARY STILL. Do not touch the pad. Measuring for %.0fs ***\n"
          % args.seconds)

    buf = b""
    t0 = time.time()
    frames = []          # (t, dists)
    lidar_stats = []     # (t, pkts, bad)

    while time.time() - t0 < args.seconds:
        if not getattr(src, "in_waiting", False):
            time.sleep(0.005)
            continue
        chunk = src.readline()
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            txt = line.decode("utf-8", errors="replace").strip()

            m = LIDAR_RE.search(txt)
            if m:
                lidar_stats.append((time.time() - t0, int(m.group(2)), int(m.group(3))))
                continue

            m = SCAN_RE.match(txt)
            if not m:
                continue
            try:
                d = [int(x) for x in m.group(6).split(",")]
            except ValueError:
                continue
            if len(d) == 360:
                frames.append((time.time() - t0, np.array(d, dtype=int)))
                print("\r  frames: %d   lidar-diag lines: %d   "
                      % (len(frames), len(lidar_stats)), end="", flush=True)

    print("\n")
    set_scanner_mode(src, False)
    src.close()

    if len(frames) < 10:
        print("Only %d SCAN frames. Is scanner mode on / is Gary streaming?" % len(frames))
        sys.exit(1)

    # ---- 1. actual LiDAR throughput --------------------------------------
    print("=" * 70)
    if len(lidar_stats) >= 2:
        (ta, pa, ba), (tb, pb, bb) = lidar_stats[0], lidar_stats[-1]
        dt = tb - ta
        if dt > 1:
            pps = (pb - pa) / dt
            print("  LiDAR packets/sec : %6.1f" % pps)
            print("  bad packets       : %6d  (+%d during this run)" % (bb, bb - ba))
            print("  A healthy LDS-style LiDAR is ~2000-4000 POINTS/sec. If the packet")
            print("  rate here is ~20/s you are getting roughly a tenth of the data.")
    else:
        print("  (no LIDAR diag lines seen over TCP -- that stat is Serial-only)")

    # ---- 2. frame rate + churn -------------------------------------------
    ts = np.array([f[0] for f in frames])
    D = np.stack([f[1] for f in frames])
    dt_frame = float(np.median(np.diff(ts)))
    print()
    print("  SCAN frames/sec   : %6.2f   (%.0f ms apart)" % (1.0 / dt_frame, dt_frame * 1000))

    churn = [(D[i] != D[i - 1]).mean() for i in range(1, len(D))]
    print("  churn per frame   : %5.1f%% of the 360 bearings changed since the last frame"
          % (100 * float(np.mean(churn))))

    # ---- 3. refresh time (the headline) -----------------------------------
    # First version of this demanded that EVERY one of the 360 bearings change,
    # and so it never completed -- because a bearing that gets no return sits at
    # 0 forever and 0 != 0 is False. Those bearings are DEAD, not slow, and
    # waiting for them is waiting for nothing. So: identify the dead ones, then
    # measure the refresh time over the live ones only.
    ever_changed = (D != D[0]).any(axis=0)
    dead = int((~ever_changed).sum())
    live = np.where(ever_changed)[0]

    print("  dead bearings     : %3d of 360 never changed at all over %.0fs"
          % (dead, ts[-1] - ts[0]))
    if dead:
        zeros = int((D[:, ~ever_changed] == 0).all(axis=0).sum())
        print("                      (%d of those are stuck at 0 = no return ever)" % zeros)

    # "Time until every live bearing has CHANGED VALUE" looks like the obvious
    # staleness metric. It is not, and it misled us twice before I caught it.
    # Gary is standing still, staring at a static wall: a bearing that keeps
    # correctly reporting 548mm forever never "changes" -- so this metric brands
    # the most TRUSTWORTHY numbers in the ring as the most stale. It measures
    # scene activity, not data freshness. Computed below out of curiosity only.
    # Do not steer by it.
    refresh = []
    for i in range(0, len(D) - 1):
        seen = np.zeros(len(live), dtype=bool)
        for j in range(i + 1, len(D)):
            seen |= (D[j][live] != D[i][live])
            if seen.all():
                refresh.append(ts[j] - ts[i])
                break

    # THE SOUND METRIC: churn. What fraction of the ring visibly turns over
    # between consecutive frames? A ring that is a genuine snapshot is being
    # completely repainted by the spinning LiDAR many times a second, so sensor
    # noise alone flips most bearings every single frame. A ring that is a slow
    # mosaic barely moves, because most of its numbers are simply OLD.
    churn_pct = 100.0 * float(np.mean(churn))
    print()
    print("  CHURN per frame   : %.1f%%      <-- THE NUMBER THAT MATTERS" % churn_pct)
    print("                      (fraction of the ring turning over every %.0f ms)"
          % (dt_frame * 1000))
    if refresh:
        print("  [unreliable]        every-live-bearing-changed: %.2f s -- bad metric, see code"
              % float(np.median(refresh)))

    print("=" * 70)
    print()
    if churn_pct > 50.0:
        print("  VERDICT: the ring is a SNAPSHOT. The LiDAR is repainting essentially all")
        print("  of it between frames, so scan-matching against it is sound.")
        print("  Go run gary_odom_fit.py.")
    elif churn_pct > 25.0:
        print("  VERDICT: BORDERLINE. It turns over, but not fast enough to be confident.")
        print("  Check the LIDAR line on SERIAL (it is not sent over TCP): you want")
        print("  ~380+ pkts/sec and bad=0.")
    else:
        print("  VERDICT: the ring is a SMEAR, not a snapshot. Most numbers in any given")
        print("  SCAN frame are simply OLD -- carried over from earlier rotations because")
        print("  the LiDAR stream is not being fully ingested.")
        print()
        print("  This breaks far more than the calibration:")
        print("    * maps are smeared by however far Gary walked while the ring filled")
        print("    * mover-detection compares live returns against stale bearings")
        print("    * frame-to-frame ICP sees ~no motion -- the odometry fit returns ZEROS")
        print()
        print("  Check the LIDAR line on SERIAL. A healthy D500 gives ~380 pkts/sec and")
        print("  bad=0. If you see ~20 pkts/sec with bad climbing, the UART is overflowing:")
        print("  the RX buffer is too small and/or nothing drains it often enough.")
        print("  See the reader-task autopsy comment in Gary_lidar.cpp.")
    print()



if __name__ == "__main__":
    main()
