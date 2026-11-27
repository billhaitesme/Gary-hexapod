#!/usr/bin/env python3
"""
gary_pose_diag.py -- show ONLY the pose fields of the SCAN stream, live.

WHY THIS EXISTS
The live map has now been wrong three times, and each time I diagnosed it from a
screenshot and each time I was wrong. A smeared map, a mis-signed yaw, a
mis-signed tilt and a dead sensor all produce a blob, and a blob is a blob. So
stop looking at the blob. Look at the four numbers that BUILD the blob.

Drive Gary and watch:

  THETA  must track his real heading. Spin him 90 deg right -- theta should move
         by 90. If it stays 0.0, the gyro is off. If it moves the WRONG WAY,
         GYRO_YAW_SCALE needs a minus sign. If it moves 57x too far, the driver
         is handing back rad/s.
  PITCH/ROLL  must be ~0 when he's level and swing when you work the right stick.
         If they're pinned at 0.0, _imu_update() still isn't running.
  X/Y    should grow when he walks and NOT grow when he's standing still. If they
         creep while he's parked, odometry is integrating phantom motion again.

Everything else in the map is downstream of these four. Get them right and the
map is right; get them wrong and no amount of dashboard maths will save it.

    python gary_pose_diag.py --source tcp:192.168.1.156:3333
"""
import argparse
import re
import sys
import time

from gary_lidar_io import open_source, set_scanner_mode

SCAN_RE = re.compile(
    r"^SCAN:(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)"
    r"(?:,(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?))?;")
GYRO_RE = re.compile(r">>> GYRO \| x=(-?[\d.]+) y=(-?[\d.]+) z=(-?[\d.]+)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", required=True)
    args = ap.parse_args()

    src = open_source(args.source)
    set_scanner_mode(src, True)
    print("Connected. DRIVE HIM. Turn him ~90 deg and watch THETA.")
    print()
    print("      x_cm    y_cm   theta   pitch    roll   |  gyro_z (deg/s)")
    print("  " + "-" * 62)

    gz = float("nan")
    n = 0
    t0 = time.time()
    theta0 = None
    try:
        while True:
            # TCPLineSource exposes readline(), not iteration -- match how every
            # other script in this folder drains it.
            raw = src.readline()
            if not raw:
                time.sleep(0.005)
                continue
            txt = raw.decode("utf-8", "replace").strip() if isinstance(raw, bytes) else raw.strip()

            g = GYRO_RE.search(txt)
            if g:
                gz = float(g.group(3))

            m = SCAN_RE.match(txt)
            if not m:
                continue
            x, y, th = float(m.group(1)), float(m.group(2)), float(m.group(3))
            pitch = float(m.group(4)) if m.group(4) is not None else float("nan")
            roll = float(m.group(5)) if m.group(5) is not None else float("nan")
            if theta0 is None:
                theta0 = th
            n += 1
            if n % 3:            # ~1.5 lines/sec, readable
                continue
            print("  %8.1f %7.1f %7.1f %7.1f %7.1f   | %8.2f"
                  % (x, y, th, pitch, roll, gz))
    except KeyboardInterrupt:
        pass

    print()
    if theta0 is not None:
        print("  net heading change over the run: %.1f deg" % (th - theta0))
    print("  If THETA never moved, the gyro is still off -- check that setup() calls")
    print("  gary.board.imu_update(true). If it moved the wrong way or by a wild")
    print("  factor, that is GYRO_YAW_SCALE, and it is one number.")


if __name__ == "__main__":
    main()
