"""Corridor-bar probe: start south of cbar2 (22 mm, y=0.90) in the B-C
corridor, walk straight north with the climb reflex, dump sonar/height/pos.
Run from sim/:  uv run --with mujoco --with numpy python out/probe_bar.py
"""
import math, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from collections import deque
import numpy as np
import mujoco
import gary_mujoco as G
import nav_maze as NV

model = mujoco.MjModel.from_xml_string(G.build_xml(maze=True, corr_obs=True))
data = mujoco.MjData(model)
data.qpos[0], data.qpos[1], data.qpos[2] = 0.875, 0.45, G.COXA_Z
drv = G.GaitDriver("TRIPOD", 1.0, lift_cm=2.15, cadence_ms=NV.CAD, speed_cap=1.0,
                   touchdown="match")
sonar_hist = deque(maxlen=NV.RING_WIN)
climb_until = -1.0
t, nxt = 0.0, 0.0
for s in range(int(60.0 / model.opt.timestep)):
    if s % NV.CTRL_EVERY == 0:
        sd = data.sensordata
        sonar_hist.append(2.0 if sd[0] < 0 else float(sd[0]))
        sonar = max(sonar_hist)
        yawr = NV.yaw_of(data.body("base").xquat)
        if t > 1.0:
            sonar_floor = (0.0403 + drv.height) / 0.1736
            if 0.10 < sonar < sonar_floor - 0.05:
                climb_until = t + 25.0
            climbing = t < climb_until
            drv.v = G.WALK_SPEED_CM_S * 0.01 * (0.9 if climbing else 0.85)
            drv.omega = float(np.clip(-1.6*NV.wrap(yawr), -0.42, 0.42))
            drv.lift = 0.030 if climbing else 0.022
            drv.height = 0.030 if climbing else 0.020
        if t >= nxt:
            print(f"t={t:5.1f} pos=({data.body('base').xpos[0]:+.2f},{data.body('base').xpos[1]:+.2f})"
                  f" z={data.body('base').xpos[2]:.3f} sonar={sonar:.3f}"
                  f" climb={t < climb_until}")
            nxt = t + 1.0
    data.ctrl[:] = drv.ctrl(model.opt.timestep)
    mujoco.mj_step(model, data)
    t += model.opt.timestep
    if data.body("base").xpos[1] > 1.25:
        print("CROSSED at t=", round(t, 1))
        break
    if data.body("base").xpos[2] < 0.02:
        print("FELL at t=", round(t, 1))
        break
