"""Stage-3b: Gary trains HIMSELF to cross the obstacle course (around / over / under).

The course (gary_mujoco.build_xml(obstacles=True), along +y):
    y=0.35  bar1   12 mm tall  (step over — or drive around, |x| > 0.21)
    y=0.70  bar2   22 mm tall  (step over needs lift>=2.2 cm — or around)
    y=1.15  bridge underside 119 mm (standing Gary+LiDAR is 126 mm -> must crouch — or around)
    finish  y > 1.45

The POLICY is a 15-number vector — for each of the three obstacle zones:
    [x_aim, speed, body_height, foot_lift, gait]
A simple reflex layer turns that into commands each tick: strafe P-steers toward
x_aim, a yaw P-hold keeps him pointed +y (stays under the sim's 0.45 rad/s yaw
ceiling), speed/height/lift/gait come straight from the active zone's numbers.
CMA-ES then optimizes the whole vector on physics rollouts. Nothing tells it
"crouch at the bridge" — if crouching (or swerving around) wins, it learns it.

Score (lower=better): -100*progress_y  +8*minutes  +0.10*knee_sat%  +40*fell.
Knee-saturation is the burnout proxy — abuse costs progress.

Run:   uv run --with mujoco --with numpy --with cma python train_course.py --gens 14 --pop 8
Replay best (after training):
       uv run --with mujoco --with numpy python train_course.py --replay            (live window)
       uv run --with mujoco --with numpy --with imageio --with imageio-ffmpeg \\
           python train_course.py --replay --video out/gary_course.mp4
"""
import argparse, json, math, os, time
import numpy as np
import mujoco
import gary_mujoco as G

HERE = os.path.dirname(os.path.abspath(__file__))
BEST_PATH = os.path.join(HERE, "out", "course_policy.json")

ZONES = [0.50, 0.95, 1.45]           # zone ends (y); zone = first end > body_y
FINISH_Y = 1.45
SIM_MAX_S = 45.0
CTRL_EVERY = 5                        # reflex layer at 100 Hz (physics 500 Hz)
TRAINED = {"TRIPOD": (493, 2.15), "WAVE": (911, 1.09), "RIPPLE": (416, 0.76)}
GAITS = list(TRAINED)

# per-zone knob bounds: x_aim, speed, height, lift, gait-selector
LO = np.array([-0.32, 0.15, -0.022, 0.006, 0.0])
HI = np.array([ 0.32, 1.20,  0.045, 0.030, 2.999])

def decode(x):
    """15 numbers in [0,1] -> 3 zones x 5 knobs."""
    x = np.clip(np.asarray(x, dtype=float).reshape(3, 5), 0, 1)
    return LO + x * (HI - LO)

def yaw_of(data):
    q = data.body("base").xquat
    return math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))

def rollout(params, view=False, video=None, verbose=False):
    model = mujoco.MjModel.from_xml_string(G.build_xml(obstacles=True))
    data = mujoco.MjData(model)
    data.qpos[2] = G.COXA_Z
    zone_knobs = decode(params)
    gait_idx = -1
    drv = None
    knee_ids = [3*i+2 for i in range(6)]
    sat_steps = steps = 0
    t = 0.0
    last_prog, last_prog_t = 0.0, 0.0
    frames = []
    renderer = mujoco.Renderer(model, 540, 960) if video else None
    viewer = None
    if view:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data)
    wall0 = time.perf_counter()
    n = int(SIM_MAX_S / model.opt.timestep)
    result = None
    for s in range(n):
        by = data.body("base").xpos[1]
        z = min(int(np.searchsorted(ZONES, by, side="right")), 2)
        x_aim, speed, height, lift, gsel = zone_knobs[z]
        gi = min(2, int(gsel))
        if gi != gait_idx or drv is None:
            gait_idx = gi
            cad, _ = TRAINED[GAITS[gi]]
            nd = G.GaitDriver(GAITS[gi], 1.0, lift_cm=2.0, cadence_ms=cad,
                              speed_cap=1.0, touchdown="match")
            if drv is not None:
                nd.t = drv.t
            drv = nd
        if s % CTRL_EVERY == 0:
            bx = data.body("base").xpos[0]
            yw = yaw_of(data)
            drv.v = G.WALK_SPEED_CM_S * 0.01 * speed
            drv.vx = float(np.clip(2.5 * (x_aim - bx), -0.03, 0.03))   # strafe P-steer
            drv.omega = float(np.clip(-1.5 * yw, -0.4, 0.4))          # yaw P-hold, safe band
            drv.height = float(height)
            drv.lift = float(lift)
        data.ctrl[:] = drv.ctrl(model.opt.timestep)
        mujoco.mj_step(model, data)
        t += model.opt.timestep
        steps += 1
        f = np.abs(data.actuator_force[knee_ids])
        sat_steps += int((f > G.SERVO_TORQUE * 0.95).any())
        bz = data.body("base").xpos[2]
        fell = bz < 0.028 or abs(math.degrees(yaw_of(data))) > 80
        prog = max(last_prog, by)
        if prog > last_prog + 0.01:
            last_prog, last_prog_t = prog, t
        stuck = (t - last_prog_t) > 7.0
        if viewer is not None:
            if not viewer.is_running():
                break
            viewer.sync()
            lag = (wall0 + t) - time.perf_counter()
            if lag > 0:
                time.sleep(lag)
        if renderer is not None and s % 16 == 0:
            cam = mujoco.MjvCamera()
            cam.lookat[:] = data.body("base").xpos
            cam.distance, cam.elevation, cam.azimuth = 0.55, -18, 120
            renderer.update_scene(data, camera=cam)
            frames.append(renderer.render().copy())
        if by >= FINISH_Y or fell or stuck:
            result = dict(done=by >= FINISH_Y, fell=bool(fell), stuck=bool(stuck))
            break
    if viewer is not None:
        viewer.close()
    if result is None:
        result = dict(done=False, fell=False, stuck=False)
    prog = float(max(last_prog, data.body("base").xpos[1]))
    sat_pct = 100.0 * sat_steps / max(steps, 1)
    score = (-100.0 * prog + 8.0 * (t / 60.0) + 0.10 * sat_pct
             + (40.0 if result["fell"] else 0.0))
    result.update(progress_y=round(prog, 3), time_s=round(t, 1),
                  knee_sat_pct=round(sat_pct, 1), score=round(score, 2))
    if verbose:
        print(result, flush=True)
    if video and frames:
        import imageio
        imageio.mimwrite(video, frames, fps=30, quality=8)
        result["video"] = video
    return score, result

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gens", type=int, default=14)
    ap.add_argument("--pop", type=int, default=8)
    ap.add_argument("--replay", action="store_true")
    ap.add_argument("--video", default=None)
    a = ap.parse_args()

    if a.replay:
        best = json.load(open(BEST_PATH))
        _, r = rollout(best["params"], view=not a.video, video=a.video, verbose=True)
        print(json.dumps(r, indent=1))
        return

    import cma
    # seed: straight line, medium speed, neutral height, trained-ish lift, tripod
    x0 = np.tile([0.5, 0.55, 0.33, 0.55, 0.9], 3)
    es = cma.CMAEvolutionStrategy(x0.tolist(), 0.28,
                                  {"popsize": a.pop, "bounds": [0, 1], "seed": 11, "verbose": -9})
    best = (1e9, None, None)
    for gen in range(a.gens):
        xs = es.ask()
        losses = []
        for x in xs:
            sc, r = rollout(x)
            losses.append(sc)
            if sc < best[0]:
                best = (sc, list(map(float, x)), r)
        es.tell(xs, losses)
        b = best[2]
        print(f"gen {gen+1}/{a.gens} best={best[0]:.1f} "
              f"prog={b['progress_y']}m t={b['time_s']}s sat={b['knee_sat_pct']}% "
              f"done={b['done']}", flush=True)
    os.makedirs(os.path.dirname(BEST_PATH), exist_ok=True)
    knobs = decode(best[1])
    readable = [dict(zone=i, x_aim=round(float(k[0]), 3), speed=round(float(k[1]), 2),
                     height_mm=round(float(k[2] * 1000), 1), lift_cm=round(float(k[3] * 100), 2),
                     gait=GAITS[min(2, int(k[4]))]) for i, k in enumerate(knobs)]
    json.dump(dict(params=best[1], metrics=best[2], policy=readable),
              open(BEST_PATH, "w"), indent=1)
    print("WROTE", BEST_PATH)
    print(json.dumps(dict(metrics=best[2], policy=readable), indent=1))

if __name__ == "__main__":
    main()
