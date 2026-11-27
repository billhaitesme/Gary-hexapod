"""MOTION AUDIT — the machine version of Bill's eyes (his directive:
"help me fix it, stop assuming, and stop relying on me to catch everything").
Runs a drill and reports PER-PHASE truth: pitch held vs peaked, whether the
front feet were actually airborne during "liberation", belly contact the
tick it starts, real forward progress. Claims about motion get made by THIS
report and frame reads — not by scalar peaks, never again by charity.

Run: uv run --with "mujoco<3.12" --with numpy python motion_audit.py --drill rear
"""
import argparse, json, math
import mujoco
import gary_mujoco as G
import door_gym as DG


def audit(drill="rear", genes=None):
    log = []
    model = [None]

    # piggyback run_drill via frame-less instrumentation: re-run its loop
    # ingredients here so the audit sees raw mjData every control tick
    import numpy as np
    m = mujoco.MjModel.from_xml_string(G.build_xml(
        door_gym=(drill not in ("wall", "jump", "rear")),
        wall_gym=(drill in ("wall", "rear"))))
    d = mujoco.MjData(m)
    badr = int(m.jnt_qposadr[m.body("base").jntadr[0]])
    d.qpos[badr + 2] = G.COXA_Z
    d.qpos[badr + 1] = 0.235
    feet = [m.geom(f"foot{i}").id for i in range(6)]
    nose = m.site("ld0").id
    floor = m.geom("floor").id if True else None
    drv = G.GaitDriver(); drv.v = drv.vx = 0.0; drv.omega = 0.0; drv.lift = 0.0
    t, s = 0.0, 0
    # reuse door_gym's control policy by simulating its branch inline is
    # fragile — instead call its run_drill with a spy callback? run_drill
    # owns the loop; simplest robust: replicate ONLY the rear drill here.
    assert drill == "rear", "audit currently covers the rear drill"
    while t < 18.0:
        if s % 5 == 0:
            if t < 1.0:
                pass
            elif t < 4.6:
                drv.v = 0.058
                drv.lift = 0.016     # WITH LIFT — zero-lift creep was a
                # shuffle-in-place (commanded 21 cm, delivered 2: the plow
                # again); he has been stranded 19 cm out this whole time
                # because the APPROACH never worked
                # CREEP CLOSE — Bill's STEP ONE, and
                # the step I kept shorting: every ram launched from 19 cm
                # out (three body-lengths!). Arrive at ~7 cm; ONE ram then
                # puts the flying fronts ONTO the wall, and recovery
                # happens under WALL support — a different stability world.
            if t >= 4.6:
                rp = min(1.0, (t - 4.6) / 1.6)
                # NEVER FULLY SIT: at fold -0.044 the rear hips reach
                # floor level, the butt takes the load, and un-sitting
                # means dead-lifting the aft with clutch-limited servos —
                # impossible; every walk failed because he was SEATED.
                # The held pose was 12 deg either way (18 was the sit-down
                # transient). Hover the butt ~1 cm proud and keep it.
                drv.leg_h[4] = drv.leg_h[5] = -0.028 * rp
                drv.leg_h[0] = drv.leg_h[3] = 0.042 * rp
                drv.leg_h[1] = drv.leg_h[2] = 0.055 * rp
                drv.body_dy = -0.042 * rp   # FULL AFT COCK (Bill: the
                # controller-era whole-body shift IS the ram — all hip
                # zeros slide together; cock deep = full runway)
                drv.v = 0.0
            if t >= 4.6:
                drv.v = 0.0
                drv.lift = 0.0
            if t > 6.6:
                # BILL'S LUNGE-STROKE (verbatim spec): fronts lift; AT
                # APEX the mids row backward at the hip and the rears
                # extend — one coordinated push-off AT the wall while the
                # fronts are out; on contact the back four RECOVER
                # UNDERNEATH to pin him against it and keep him coming.
                # A swimmer's stroke, not a gait — no gait to collapse.
                drv.press = 1
                drv.press_hi = 0.032
                if t <= 7.0:                       # first rise to apex
                    drv.v = 0.0
                else:
                    # THE SWIM: chained cock-RAM-step cycles (one ram nets
                    # +2.2 cm with the nose RISING and pitch holding 12 —
                    # the wall needs six of them)
                    _cyc = (t - 7.0) % 2.15
                    drv.v = 0.0                    # NO GAIT, EVER — the
                    drv.lift = 0.0                 # walk machinery has
                    # flattened every pose tonight; the swimmer recovers
                    # one limb at a time instead (Bill's spec, literal)
                    if _cyc <= 0.35:               # RAM: hips sweep back
                        _sp = _cyc / 0.35          # while ALL FOUR legs
                        _ext = 0.020 * min(1.0, _sp * 1.6)
                        drv.leg_h[4] = drv.leg_h[5] = -0.028 + _ext
                        drv.leg_h[0] = drv.leg_h[3] = 0.042 + _ext * 0.5
                        # extend THROUGH the sweep ("extend the legs to
                        # support the momentum and push")
                        drv.body_dy = -0.042 + 0.084 * _sp
                    elif _cyc <= 0.55:             # ZERO-SNAP: re-cock with
                        drv.body_dy = -0.042       # all feet planted —
                        drv.leg_h[4] = drv.leg_h[5] = -0.028
                        drv.leg_h[0] = drv.leg_h[3] = 0.042
                        # friction holds the chassis; only the TARGETS move
                    else:                          # LIMB-BY-LIMB RECOVERY:
                        _leg = (4, 5, 0, 3)[min(3, int((_cyc - 0.55) / 0.4))]
                        _lp2 = ((_cyc - 0.55) % 0.4) / 0.4
                        _lift2 = 0.022 * math.sin(3.14159 * _lp2)
                        drv.leg_h[4] = drv.leg_h[5] = -0.028
                        drv.leg_h[0] = drv.leg_h[3] = 0.042
                        if _leg in (4, 5):
                            drv.leg_h[_leg] = -0.028 + _lift2
                        else:
                            drv.leg_h[_leg] = 0.042 + _lift2
                        drv.body_dy = -0.042
                        # each lifted leg snaps to its forward zero and
                        # re-plants; three planted + friction hold station
        if abs(t - 5.0) < 0.0011:
            print(f'PROOF t=5: leg_h4={drv.leg_h[4]:.3f} lift={drv.lift} v={drv.v} press={drv.press}')
        d.ctrl[:] = drv.ctrl(m.opt.timestep)
        mujoco.mj_step(m, d)
        t += m.opt.timestep
        s += 1
        if s % 25 == 0:                      # 50 ms samples
            mp = DG.rp_of(d.body("base").xquat)[0]
            fg = [False] * 6
            belly = False
            for ci in range(d.ncon):
                c = d.contact[ci]
                g1, g2 = c.geom1, c.geom2
                for fi, fid in enumerate(feet):
                    if fid in (g1, g2):
                        fg[fi] = True
                n1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g1) or ""
                n2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g2) or ""
                if "floor" in (n1, n2):
                    other = n2 if n1 == "floor" else n1
                    if not other.startswith("foot"):
                        belly = True
            log.append(dict(t=round(t, 2), pitch=round(math.degrees(mp), 1),
                            base=round(float(d.body("base").xpos[2]) * 100, 1),
                            nose=round(float(d.site_xpos[nose][2]) * 100, 1),
                            y=round(float(d.body("base").xpos[1]) * 100, 1),
                            f1=fg[1], f2=fg[2], belly=belly))
    # per-phase digest
    phases = (("pose", 1.0, 3.6), ("liberate", 3.6, 4.4), ("walk", 4.4, 10.0))
    print(f"{'phase':10s} {'pitch avg/min':>14s} {'nose avg':>9s} "
          f"{'fronts grounded%':>17s} {'belly%':>7s} {'adv cm':>7s}")
    verdicts = []
    for nm, t0, t1 in phases:
        rows = [r for r in log if t0 <= r["t"] < t1]
        if not rows:
            continue
        pa = sum(r["pitch"] for r in rows) / len(rows)
        pmin = min(r["pitch"] for r in rows)
        na = sum(r["nose"] for r in rows) / len(rows)
        fgp = 100 * sum(1 for r in rows if r["f1"] or r["f2"]) / len(rows)
        bp = 100 * sum(1 for r in rows if r["belly"]) / len(rows)
        adv = rows[-1]["y"] - rows[0]["y"]
        print(f"{nm:10s} {pa:7.1f}/{pmin:5.1f} {na:9.1f} {fgp:16.0f}% "
              f"{bp:6.0f}% {adv:7.1f}")
        verdicts.append((nm, pa, fgp, bp, adv))
    # stroke zoom: 100 ms rows through the action
    print("--- stroke zoom (t 3.9-6.3) ---")
    print(f"{'t':>5s} {'pitch':>6s} {'base':>5s} {'nose':>5s} {'y':>6s} {'f1/f2 grounded':>15s}")
    for r in log:
        if 5.9 <= r["t"] <= 17.9 and (round(r["t"] * 10) % 4 == 0):
            print(f"{r['t']:5.1f} {r['pitch']:6.1f} {r['base']:5.1f} "
                  f"{r['nose']:5.1f} {r['y']:6.1f} {str(r['f1']):>7s}/{r['f2']}")
    # the belly moment
    fall = next((r for r in log if r["belly"] and r["t"] > 3.0), None)
    if fall:
        print(f"BELLY at t={fall['t']} (pitch {fall['pitch']}, "
              f"fronts grounded={fall['f1'] or fall['f2']})")
    return log


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--drill", default="rear")
    a = ap.parse_args()
    audit(a.drill)
