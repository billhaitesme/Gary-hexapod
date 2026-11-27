"""Fingerprint probe: plant Gary at candidate standpoints in the shortcut
world, run one gimbal revolution, dump per-bearing level/up/down for bearings
toward the C tunnel (1.10, 0.33). Run from sim/:
  uv run --with mujoco --with numpy python out/probe_scan.py
"""
import math, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import mujoco
import gary_mujoco as G
import nav_maze as NV

for (px, py) in [(0.95, 0.09), (0.62, 0.33), (0.95, 0.62), (0.72, 0.20)]:
    model = mujoco.MjModel.from_xml_string(G.build_xml(maze=True, shortcut=True))
    data = mujoco.MjData(model)
    data.qpos[0], data.qpos[1], data.qpos[2] = px, py, G.COXA_Z
    drv = G.GaitDriver("TRIPOD", 1.0, lift_cm=2.15, cadence_ms=NV.CAD,
                       speed_cap=1.0, touchdown="match")
    drv.v = 0.0
    down = [0.0]*24; up = [0.0]*24; level = [0.0]*24
    A = math.radians(20.0); T_g = 16.0
    t = 0.0
    n = int(19.0 / model.opt.timestep)
    for s in range(n):
        if s % NV.CTRL_EVERY == 0:
            ring = [2.0 if v < 0 else float(v) for v in data.sensordata[1:25]]
            if t > 2.0:
                ph = 2*math.pi*(t-2.0)/T_g
                drv.lift, drv.height = 0.0, 0.020
                drv.body_pitch = A*math.cos(ph); drv.body_roll = A*math.sin(ph)
                for b in range(24):
                    az = math.radians(15*b)
                    elev = drv.body_pitch*math.cos(az) - drv.body_roll*math.sin(az)
                    if -0.16 < elev < -0.07:
                        down[b] = max(down[b], ring[b])
                    elif elev > 0.09:
                        up[b] = max(up[b], ring[b])
                    elif abs(elev) < 0.03:
                        level[b] = max(level[b], ring[b])
        data.ctrl[:] = drv.ctrl(model.opt.timestep)
        mujoco.mj_step(model, data)
        t += model.opt.timestep
    yawr = NV.yaw_of(data.body("base").xquat)
    tb = math.degrees(math.atan2(1.10-px, 0.33-py))  # world bearing to tunnel
    print(f"--- standpoint ({px},{py})  yaw={math.degrees(yawr):.0f}  tunnel world-bearing {tb:.0f}")
    hits = []
    for b in range(24):
        ok = level[b] < 0.50 and down[b] > 0.55 and up[b] < 0.6 and (down[b]-up[b]) > 0.30
        if ok:
            hits.append(15*b)
        wb = (15*b - math.degrees(yawr)) % 360
        if abs(((wb - tb + 180) % 360) - 180) < 40:
            print(f"   b={15*b:3d} (world {wb:5.1f})  level={level[b]:.2f} up={up[b]:.2f} down={down[b]:.2f}  {'FINGERPRINT' if ok else ''}")
    print("   verdict bearings:", hits)
