"""The sensor experiment (Bill, 2026-08-19): is servo/contact feedback WORTH IT?

Three variants cross the same 105-bump field (4-16 mm random bumps, 1 m long):
  none    - no terrain sensing: one fixed posture, best it can evolve
  imu     - what real Gary senses TODAY: IMU jolt (pitch+roll rate) + global
            current proxy (sum of |servo torque| — the board's battery ammeter)
  perleg  - what he could sense with VsenseMod/FSRs: max per-KNEE torque EMA
Policy (7 genes, CMA-ES): smooth speed/lift, rough speed/lift/height, the rough
trigger threshold, and how long "rough" persists. Variant none disables the trigger.

The A->B score gap = value of the free (IMU) feedback stack.
The B->C gap = value of adding real per-leg sensing hardware.

Run:  uv run --with mujoco --with numpy --with cma python train_terrain_sense.py --variant imu
      (or none / perleg / all)
"""
import argparse, json, math, os
import numpy as np
import mujoco
import gary_mujoco as G

HERE = os.path.dirname(os.path.abspath(__file__))
FINISH_Y = 1.55
SIM_MAX_S = 60.0
CTRL_EVERY = 5
CAD, LIFT0 = 493, 2.15   # trained tripod

KEYS = ["smooth_speed", "smooth_lift", "rough_speed", "rough_lift", "rough_height", "trig", "hold"]
LO = np.array([0.20, 0.008, 0.10, 0.012, -0.005, 0.10, 0.2])
HI = np.array([1.20, 0.030, 0.90, 0.032,  0.045, 3.00, 2.0])
# trig units: imu -> deg/tick EMA * 10; perleg -> N*m * 10 (both scaled into 0.1-3.0)

def decode(x):
    x = np.clip(np.asarray(x, dtype=float), 0, 1)
    return dict(zip(KEYS, LO + x * (HI - LO)))

def rollout(params, variant, video=None):
    p = decode(params)
    model = mujoco.MjModel.from_xml_string(G.build_xml(bumps=True))
    data = mujoco.MjData(model)
    data.qpos[2] = G.COXA_Z
    drv = G.GaitDriver("TRIPOD", 1.0, lift_cm=LIFT0, cadence_ms=CAD, speed_cap=1.0,
                       touchdown="match")
    knee_ids = [3*i+2 for i in range(6)]
    sat = steps = 0
    t = 0.0
    rough_until = -1.0
    prev_pitch = prev_roll = 0.0
    ema = 0.0
    falls = 0
    frames = []
    renderer = mujoco.Renderer(model, 540, 960) if video else None
    n = int(SIM_MAX_S / model.opt.timestep)
    last_prog, last_prog_t = 0.0, 0.0
    for s in range(n):
        if s % CTRL_EVERY == 0:
            q = data.body("base").xquat
            pitch = math.degrees(math.asin(max(-1, min(1, 2*(q[2]*q[3] + q[0]*q[1])))))
            roll = math.degrees(math.asin(max(-1, min(1, 2*(q[0]*q[2] - q[3]*q[1])))))
            yawr = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
            f = np.abs(data.actuator_force[knee_ids])
            if variant == "imu":
                jolt = abs(pitch - prev_pitch) + abs(roll - prev_roll)   # deg per ctrl tick
                curr = float(np.abs(data.actuator_force).sum())          # global torque ~ current
                sig = 10.0 * jolt + 2.0 * curr
            elif variant == "perleg":
                sig = 10.0 * float(f.max())                              # worst knee torque
            else:
                sig = 0.0
            prev_pitch, prev_roll = pitch, roll
            ema = 0.7 * ema + 0.3 * sig
            if variant != "none" and ema > p["trig"]:
                rough_until = t + p["hold"]
            rough = t < rough_until
            drv.v = G.WALK_SPEED_CM_S * 0.01 * (p["rough_speed"] if rough else p["smooth_speed"])
            drv.lift = p["rough_lift"] if rough else p["smooth_lift"]
            drv.height = p["rough_height"] if rough else 0.0
            bx = data.body("base").xpos[0]
            drv.vx = float(np.clip(-2.0 * bx, -0.02, 0.02))
            drv.omega = float(np.clip(-1.5 * yawr, -0.4, 0.4))
        data.ctrl[:] = drv.ctrl(model.opt.timestep)
        mujoco.mj_step(model, data)
        t += model.opt.timestep
        steps += 1
        sat += int((np.abs(data.actuator_force[knee_ids]) > G.SERVO_TORQUE * 0.95).any())
        by = data.body("base").xpos[1]
        bz = data.body("base").xpos[2]
        if bz < 0.022:
            falls += 1
        if by > last_prog + 0.01:
            last_prog, last_prog_t = by, t
        if renderer is not None and s % 16 == 0:
            cam = mujoco.MjvCamera(); cam.lookat[:] = data.body("base").xpos
            cam.distance, cam.elevation, cam.azimuth = 0.5, -20, 130
            renderer.update_scene(data, camera=cam)
            frames.append(renderer.render().copy())
        if by >= FINISH_Y or (t - last_prog_t) > 10.0:
            break
    prog = float(max(last_prog, data.body("base").xpos[1]))
    sat_pct = 100.0 * sat / max(steps, 1)
    stumble_s = falls * model.opt.timestep
    score = -100.0 * prog + 8.0 * (t / 60.0) + 0.10 * sat_pct + 5.0 * stumble_s
    r = dict(variant=variant, done=bool(prog >= FINISH_Y), progress_y=round(prog, 3),
             time_s=round(t, 1), knee_sat_pct=round(sat_pct, 1),
             stumble_s=round(stumble_s, 2), score=round(score, 2))
    if video and frames:
        import imageio
        imageio.mimwrite(video, frames, fps=30, quality=8)
        r["video"] = video
    return score, r

def train(variant, gens, pop):
    import cma
    es = cma.CMAEvolutionStrategy([0.5]*7, 0.28, {"popsize": pop, "bounds": [0, 1],
                                                  "seed": 3, "verbose": -9})
    best = (1e9, None, None)
    for gen in range(gens):
        xs = es.ask()
        losses = []
        for x in xs:
            sc, r = rollout(x, variant)
            losses.append(sc)
            if sc < best[0]:
                best = (sc, list(map(float, x)), r)
        es.tell(xs, losses)
        print(f"[{variant}] gen {gen+1}/{gens} best={best[0]:.1f} "
              f"prog={best[2]['progress_y']} t={best[2]['time_s']}s "
              f"sat={best[2]['knee_sat_pct']}% done={best[2]['done']}", flush=True)
    return best

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--variant", default="all", choices=["none", "imu", "perleg", "all"])
    ap.add_argument("--gens", type=int, default=12)
    ap.add_argument("--pop", type=int, default=8)
    a = ap.parse_args()
    variants = ["none", "imu", "perleg"] if a.variant == "all" else [a.variant]
    out = {}
    for v in variants:
        sc, params, r = train(v, a.gens, a.pop)
        out[v] = dict(score=round(sc, 2), params=params, metrics=r,
                      policy={k: round(float(val), 4) for k, val in decode(params).items()})
    os.makedirs(os.path.join(HERE, "out"), exist_ok=True)
    json.dump(out, open(os.path.join(HERE, "out", "terrain_sense.json"), "w"), indent=1)
    print(json.dumps({v: d["metrics"] for v, d in out.items()}, indent=1))
