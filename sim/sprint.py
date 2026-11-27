"""OPEN-GROUND SPRINT — the drag strip (Bill: "see what it can do given no
obstacles... in case while it's wandering around it ever wants a case of the
zoomies").

Flat floor, standing start, 2.5 m dash. Sweeps the pure gait envelope —
stride multiplier x cadence x lift — with no navigation in the way, and
reports for every cell: dash time, average speed, knee saturation (fraction
of ticks any knee sits at the HPD-0127 clutch limit), lateral veer, falls.

The winners feed two things: the navigator's ZOOMIES mode (open straight
ahead -> shift up) and September's firmware sprint profile.

Run:  uv run --with mujoco --with numpy python sprint.py            (sweep)
      ... --best                     (re-run + print the saved best only)
      ... --video out/x.mp4 --stride 2.5 --cadence 420 --lift 0.012 (one dash)
"""
import argparse, itertools, json, math, os
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
DIST = 2.5
NOMINAL = 0.035          # m/s — firmware WALK_SPEED_CM_S


def dash(task):
    stride, cadence, lift = task
    import numpy as np
    import mujoco
    import gary_mujoco as G

    def yaw_of(q):
        return math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))

    model = mujoco.MjModel.from_xml_string(G.build_xml())
    data = mujoco.MjData(model)
    data.qpos[2] = G.COXA_Z
    drv = G.GaitDriver("TRIPOD", 1.0, lift_cm=lift*100, cadence_ms=cadence,
                       speed_cap=1.0, touchdown="match")
    drv.height = 0.020
    drv.lift = lift
    sat = n = 0
    t, t_launch = 0.0, 1.0           # settle, then launch
    veer_max = 0.0
    result = dict(stride=stride, cadence=cadence, lift=round(lift, 3))
    for s in range(int(150.0 / model.opt.timestep)):
        if s % 5 == 0:
            yawr = yaw_of(data.body("base").xquat)
            drv.v = NOMINAL * stride if t > t_launch else 0.0
            drv.omega = float(max(-0.3, min(0.3, -1.8 * yawr)))   # hold the lane
        data.ctrl[:] = drv.ctrl(model.opt.timestep)
        mujoco.mj_step(model, data)
        t += model.opt.timestep
        if t > t_launch:
            kf = np.abs(data.actuator_force[2::3])
            sat += int((kf > G.SERVO_TORQUE * 0.95).any())
            n += 1
        veer_max = max(veer_max, abs(float(data.body("base").xpos[0])))
        if data.body("base").xpos[2] < 0.02:
            result.update(outcome="FELL", t=round(t - t_launch, 1))
            return result
        if data.body("base").xpos[1] >= DIST:
            dt = t - t_launch
            result.update(outcome="ok", t=round(dt, 1),
                          speed_cms=round(100 * DIST / dt, 2),
                          knee_sat=round(sat / max(n, 1), 3),
                          veer_m=round(veer_max, 2))
            return result
    result.update(outcome="timeout", t=150.0)
    return result


def main():
    strides = [1.0, 1.5, 2.0, 2.5, 3.0, 3.5]
    cadences = [350, 420, 490, 560]
    lifts = [0.010, 0.017]
    tasks = list(itertools.product(strides, cadences, lifts))
    print(f"drag strip: {len(tasks)} dashes over {DIST} m", flush=True)
    rows = []
    with ProcessPoolExecutor(max_workers=6) as pool:
        for r in pool.map(dash, tasks):
            rows.append(r)
            tag = (f"{r['t']:5.1f}s  {r.get('speed_cms', 0):5.2f} cm/s  "
                   f"sat={r.get('knee_sat', 0):.2f}  veer={r.get('veer_m', 0):.2f}"
                   if r["outcome"] == "ok" else r["outcome"])
            print(f"  stride {r['stride']:.1f}  cad {r['cadence']}  "
                  f"lift {r['lift']:.3f}  ->  {tag}", flush=True)
    ok = [r for r in rows if r["outcome"] == "ok"]
    legal = [r for r in ok if r["knee_sat"] <= 0.15 and r["veer_m"] <= 0.30]
    best_any = max(ok, key=lambda r: r["speed_cms"]) if ok else None
    best_leg = max(legal, key=lambda r: r["speed_cms"]) if legal else None
    out = dict(dist_m=DIST, rows=rows, best_any=best_any, best_knee_legal=best_leg)
    with open(os.path.join(HERE, "out", "sprint.json"), "w") as f:
        json.dump(out, f, indent=1)
    print("\nBEST ANY:       ", json.dumps(best_any))
    print("BEST KNEE-LEGAL:", json.dumps(best_leg))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--stride", type=float, default=None)
    ap.add_argument("--cadence", type=int, default=490)
    ap.add_argument("--lift", type=float, default=0.012)
    ap.add_argument("--video", default=None)
    a = ap.parse_args()
    if a.stride is not None and a.video:
        # single filmed dash
        import numpy as np
        import mujoco
        import gary_mujoco as G
        model = mujoco.MjModel.from_xml_string(G.build_xml())
        data = mujoco.MjData(model)
        data.qpos[2] = G.COXA_Z
        drv = G.GaitDriver("TRIPOD", 1.0, lift_cm=a.lift*100, cadence_ms=a.cadence,
                           speed_cap=1.0, touchdown="match")
        drv.height, drv.lift = 0.020, a.lift
        renderer = mujoco.Renderer(model, 540, 960)
        frames = []
        t = 0.0

        def yaw_of(q):
            return math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
        for s in range(int(150.0 / model.opt.timestep)):
            if s % 5 == 0:
                yawr = yaw_of(data.body("base").xquat)
                drv.v = NOMINAL * a.stride if t > 1.0 else 0.0
                drv.omega = float(max(-0.3, min(0.3, -1.8 * yawr)))
            data.ctrl[:] = drv.ctrl(model.opt.timestep)
            mujoco.mj_step(model, data)
            t += model.opt.timestep
            if s % 16 == 0:
                cam = mujoco.MjvCamera()
                cam.lookat[:] = data.body("base").xpos
                cam.distance, cam.elevation, cam.azimuth = 0.9, -25, 120
                renderer.update_scene(data, camera=cam)
                frames.append(renderer.render().copy())
            if data.body("base").xpos[1] >= DIST or data.body("base").xpos[2] < 0.02:
                break
        import imageio
        imageio.mimwrite(a.video, frames, fps=30, quality=8)
        print(f"dash done t={t-1.0:.1f}s -> {a.video}")
    else:
        main()
