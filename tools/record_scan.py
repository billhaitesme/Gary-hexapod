#!/usr/bin/env python3
"""Record raw SCAN: frames from Gary to a log file for later replay through
gary_slam.py.  Usage: python record_scan.py <source> <seconds> <outfile>"""
import os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gary_lidar_io import open_source, set_scanner_mode

source = sys.argv[1] if len(sys.argv) > 1 else "tcp:gary.local:3333"
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 75.0
outp = sys.argv[3] if len(sys.argv) > 3 else "run.scan"

src = open_source(source)
set_scanner_mode(src, True)
time.sleep(0.4)
n = 0
t0 = time.time()
with open(outp, "wb") as f:
    while time.time() - t0 < secs:
        try:
            line = src.readline()
        except Exception:
            break
        if line:
            f.write(line)
            if line.startswith(b"SCAN:"):
                n += 1
try:
    set_scanner_mode(src, False); src.close()
except Exception:
    pass
print("recorded %d SCAN frames over %.0fs -> %s" % (n, secs, outp))
