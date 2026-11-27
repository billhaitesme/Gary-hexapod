"""BALANCE GYM — Bill's curriculum (2026-08-22): "start him on a platform
that gimbles... pitches and rolls in varying directions, at graduating
degrees, all the while Gary must maintain an upright position... that way it
may help in rough terrain so he can compensate for sudden changes in
slope/grade."

Stage A (this file, --stage a): the plate is DRIVEN — two mixed sines at
unequal frequencies (a Lissajous, so the tilt direction keeps wandering) with
amplitude ramping ~1 deg every 12 s until he falls. Gary stands planted and
runs an IMU counter-pose servo: body_roll/body_pitch lean INTO the plate's
tilt so his body (and the lidar ring) stays level. Genes: servo gains kp/ki,
body height (height sets how much lean authority the legs' reach allows —
Bill's "raise his allowances"), and a leak on the integrator.

Stage B (--stage b): the hinges are FREE — he must balance the plate himself
by where he puts his weight. The graduate exam. (Expect sim-flattery here:
position-servo legs; treat with September suspicion.)

Score: survival seconds (headline = the amplitude he reached), tie-broken by
mean attitude error. Falls end the run.

Run:  uv run --with "mujoco<3.12" --with numpy python balance_gym.py
      ... --watch      (Bill's windows: world w/ free cam + telemetry strip)
      ... --evolve     (evolve the servo genes)
      ... --stage b    (free-fulcrum stage)
"""
import argparse, json, math, os, time
import numpy as np
import mujoco
import gary_mujoco as G

HERE = os.path.dirname(os.path.abspath(__file__))
BAL = dict(kp=0.0, ki=0.0, leak=0.5, height=0.0,
           com_kp=0.0, com_ki=0.0, com_kd=0.0, ball_kv=0.0,
           step_gain=0.0, step_thresh=0.8, step_T=0.45, step_lift=0.014,
           tau=0.12)
BAL_BOUNDS = dict(kp=(0.0, 3.0), ki=(0.0, 2.5), leak=(0.05, 3.0),
                  height=(-0.015, 0.030),
                  com_kp=(0.0, 2.5), com_ki=(0.0, 2.0), com_kd=(0.0, 1.2),
                  ball_kv=(0.0, 3.0), step_gain=(-1.5, 1.5), step_thresh=(0.4, 0.95),
                  step_T=(0.3, 0.8), step_lift=(0.008, 0.025),
                  tau=(0.05, 0.20))
A0, RAMP, AMAX = 1.5, 1.0 / 12.0, 13.0     # deg, deg/s, cap
W1, W2 = 2*math.pi*0.29, 2*math.pi*0.203   # unequal -> wandering direction


def rp_of(q):
    """(rot_about_x, rot_about_y) in rad from a wxyz quaternion.
    IN GARY'S FRAME (x=right, y=forward): rotation about x = PITCH (nose
    up/down), rotation about y = ROLL (right side down). The first build
    labeled these the aviation way and fed pitch error into the roll
    channel — the servo fought the wrong axis and evolution's verdict was
    kp=0 ("turn it off"). Returns (pitch_like, roll_like) in DRIVER terms:
    [0] pairs with body_pitch, [1] pairs with body_roll."""
    w, x, y, z = q
    rot_x = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    rot_y = math.asin(max(-1.0, min(1.0, 2*(w*y - x*z))))
    return rot_x, rot_y


def run_balance(tune=None, stage="a", max_s=300.0, verbose=True,
                frame_cb=None, cam_ctl=None, telem_cb=None):
    bal = {**BAL, **(tune or {})}
    model = mujoco.MjModel.from_xml_string(
        G.build_xml(balance=(stage != "c"), balance_ball=(stage == "c")))
    data = mujoco.MjData(model)
    badr = int(model.jnt_qposadr[model.body("base").jntadr[0]])
    bdof = int(model.jnt_dofadr[model.body("base").jntadr[0]])
    # base freejoint address RESOLVED, not assumed: worlds whose moving
    # bodies (gimbal hinges) precede the robot shift the freejoint in qpos —
    # writing qpos[2] blindly set his X and spawned him on the floor beside
    # the pedestal, insta-"falling" at t=0
    if stage == "c":
        # THE BALL: free sphere under free plate; spawn on the stack.
        # Hold addresses for the mount steady-hand (freejoints, 7 qpos/6 dof)
        data.qpos[badr + 2] = G.COXA_Z + 0.195
        p_q = int(model.jnt_qposadr[model.body("bplate").jntadr[0]])
        p_d = int(model.jnt_dofadr[model.body("bplate").jntadr[0]])
        b_q = int(model.jnt_qposadr[model.body("bball").jntadr[0]])
        b_d = int(model.jnt_dofadr[model.body("bball").jntadr[0]])
        hold0 = (data.qpos[p_q:p_q+7].copy(), data.qpos[b_q:b_q+7].copy())
        xadr = xdof = yadr = ydof = None
        fall_z = 0.20
    else:
        data.qpos[badr + 2] = G.COXA_Z + 0.062     # spawn ON the raised plate
        # the plate is an ISLAND (pedestal): the first build sat 16 mm off
        # the floor and he "survived" 300 s by straddling — half his feet on
        # solid ground beside the plate. Stepping off the edge = falling.
        jx, jy = model.joint("gx"), model.joint("gy")
        xadr, xdof = int(jx.qposadr[0]), int(jx.dofadr[0])
        yadr, ydof = int(jy.qposadr[0]), int(jy.dofadr[0])
        fall_z = 0.075

    def plate_tilt2():
        """(tilt_x, tilt_y) of the platform, whichever platform it is."""
        if stage == "c":
            return rp_of(data.body("bplate").xquat)
        return float(data.qpos[xadr]), float(data.qpos[yadr])
    drv = G.GaitDriver(); drv.v = drv.vx = 0.0; drv.omega = 0.0
    drv.lift, drv.height = 0.0, bal["height"]
    renderer = None
    if frame_cb is not None:
        renderer = mujoco.Renderer(model, 400, 640)
    ir = ip = 0.0
    err_sum, err_n = 0.0, 0
    t, s = 0.0, 0
    wall0 = time.perf_counter()
    amp_reached = 0.0
    lvl_err = {}               # static ladder: level -> [err_sum, n]
    icp = icr = 0.0            # stage-B COM integrators
    est_prev, d_lp = [0.0, 0.0], [0.0, 0.0]
    step_until, step_v, T0 = -1.0, (0.0, 0.0), drv.T
    plate_sum, plate_n = 0.0, 0
    step_done = step_ok_since = None
    whack_mag, whack_rec, whack_ok_since = {}, {}, None
    if max_s == 300.0:         # stage-appropriate defaults
        max_s = dict(static=74.0, step=12.0).get(stage, max_s)
    while t < max_s:
        if s % 5 == 0:                          # 100 Hz control tick
            m_pitch, m_roll = rp_of(data.body("base").xquat)
            err_sum += abs(m_pitch) + abs(m_roll); err_n += 1
            dt = 0.01
            ip = ip * (1 - bal["leak"]*dt) + m_pitch*dt
            ir = ir * (1 - bal["leak"]*dt) + m_roll*dt
            # counter-pose: lean the body against what the IMU reads, AXIS
            # TO AXIS: measured rot-about-x (nose up +) is countered by
            # body_pitch (nose up +), measured rot-about-y (right down +)
            # by body_roll (right down +) — command the opposite of each.
            # SMOOTHED like the hardware (April sketch: 20 Hz + 100 ms
            # moves): a first-order lag with tau=0.12 s. Without it,
            # evolution found the sim's free lunch — 100 Hz chatter (Bill:
            # "woah, got a real jitterbug") that real servos would pay for.
            tgt_p = float(np.clip(-(bal["kp"]*m_pitch + bal["ki"]*ip),
                                  -0.30, 0.30))
            tgt_r = float(np.clip(-(bal["kp"]*m_roll + bal["ki"]*ir),
                                  -0.30, 0.30))
            alpha = dt / bal["tau"]   # loop lag as a GENE, floored at
            # the hardware-honest 20 Hz (0.05): the ball stack's 4-5 Hz
            # resonance is PUMPED by a half-cycle-late servo — the
            # 0.12 s smoothing that cured the jitterbug is lethal here
            drv.body_pitch += alpha * (tgt_p - drv.body_pitch)
            drv.body_roll += alpha * (tgt_r - drv.body_roll)
            if stage in ("b", "c", "t"):
                # STAGE B (Bill: "balance himself on a centered fulcrum"):
                # the plate is FREE — his weight placement controls it. He
                # cannot see the plate; but the attitude servo's own command
                # IS a plate-tilt estimate (it holds -plate_angle at
                # equilibrium), so the COM loop feeds off the first loop.
                # Plate right-down -> weight is right -> shift COM left;
                # plate nose-up -> rear edge down -> weight aft -> shift fwd.
                # plate = measured - commanded (body rides plate + lean):
                # command-only lagged a fast free plate; the residual term
                # carries the transient
                est_p = m_pitch - drv.body_pitch
                est_r = m_roll - drv.body_roll
                if stage == "c":
                    # BALL BRAKE (the level-plate goal was WRONG here: a
                    # level plate never decelerates a rolling ball — the
                    # stack ran away at constant speed and exited under
                    # him). Bongo doctrine: tilt AGAINST the motion. His
                    # own body velocity IS the stack velocity (IMU-honest).
                    vx_ = float(data.qvel[bdof]); vy_ = float(data.qvel[bdof+1])
                    est_p = est_p - float(np.clip(bal["ball_kv"] * vy_, -0.10, 0.10))
                    est_r = est_r - float(np.clip(-bal["ball_kv"] * vx_, -0.10, 0.10))
                    # (both signs hand-swept: this one mildly better at low
                    # kv, both hurt when strong — the coupling defeated
                    # paper analysis; evolution owns these gains now)
                dep = (est_p - est_prev[0]) / dt
                der = (est_r - est_prev[1]) / dt
                d_lp[0] += 0.2 * (dep - d_lp[0])     # rate term, low-passed:
                d_lp[1] += 0.2 * (der - d_lp[1])     # bongo-boarders react
                est_prev[0], est_prev[1] = est_p, est_r   # to tilt VELOCITY
                icp = icp * (1 - 0.4*dt) + est_p*dt
                icr = icr * (1 - 0.4*dt) + est_r*dt
                cl = 0.045 if stage == "c" else 0.028   # the ball needs
                # more COM authority than the fulcrum ever did
                tgt_dx = float(np.clip(-(bal["com_kp"]*est_r + bal["com_ki"]*icr
                                         + bal["com_kd"]*d_lp[1]), -cl, cl))
                tgt_dy = float(np.clip(+(bal["com_kp"]*est_p + bal["com_ki"]*icp
                                         + bal["com_kd"]*d_lp[0]), -cl, cl))
                drv.body_dx += alpha * (tgt_dx - drv.body_dx)
                drv.body_dy += alpha * (tgt_dy - drv.body_dy)
                # THE STEPPER (Bill: "let's get him to be a stepper" —
                # planted feet can't master a free ball): when COM-shift
                # authority is nearly spent, buy a new stance — one gait
                # burst walks the feet under the drift and recenters the
                # body between fresh footholds. step_gain is SIGNED: after
                # two sign-humblings tonight, evolution picks the direction.
                # trigger on the RUNAWAY (drift velocity), not the COM
                # offset — the offset is a lagging alarm and death arrived
                # before it ever tripped. Step direction follows velocity:
                # a rider stays UNDER the moving stack (or brakes against
                # it — the signed gene decides). Steps CHAIN while fast.
                v_mag = math.hypot(vx_, vy_) if stage == "c" else 0.0
                if (abs(bal["step_gain"]) > 0.05 and t >= step_until
                        and v_mag > bal["step_thresh"] * 0.08):
                    step_until = t + bal["step_T"]
                    step_v = (float(np.clip(bal["step_gain"] * vx_, -0.06, 0.06)),
                              float(np.clip(bal["step_gain"] * vy_, -0.06, 0.06)))
                if t < step_until:
                    drv.vx, drv.v = step_v
                    drv.lift, drv.T = bal["step_lift"], bal["step_T"]
                else:
                    drv.v = drv.vx = 0.0
                    drv.lift, drv.T = 0.0, T0
            _ptx, _pty = plate_tilt2()
            plate_sum += abs(_ptx) + abs(_pty)
            plate_n += 1
            if telem_cb is not None and s % 50 == 0:
                telem_cb(dict(t=t, amp=math.degrees(_amp(t, stage)),
                              plat=math.degrees(abs(_ptx) + abs(_pty)),
                              body=math.degrees(abs(m_pitch) + abs(m_roll))))
        if stage == "a":
            a = _amp(t, stage)
            amp_reached = max(amp_reached, a)
            data.qpos[xadr] = a * math.sin(W1 * t)
            data.qvel[xdof] = a * W1 * math.cos(W1 * t)
            data.qpos[yadr] = a * math.sin(W2 * t + 1.1)
            data.qvel[ydof] = a * W2 * math.cos(W2 * t + 1.1)
        elif stage == "c":
            # STEADYING HAND AS PHYSICS, not teleport: the qpos-freeze hold
            # corrupted foot contacts (feet skated off the frozen plate in
            # 0.4 s — proven by the no-hold control experiment, which
            # mounted cleanly). Virtual springs pull plate and ball toward
            # the mount pose, full strength while he settles, fading to
            # zero by t=6. After that: pure ball physics.
            fade = 1.0 if t < 2.0 else max(0.0, (6.0 - t) / 4.0)
            for q0, qa, da in ((hold0[0], p_q, p_d), (hold0[1], b_q, b_d)):
                if fade <= 0.0:
                    data.qfrc_applied[da:da+6] = 0.0
                    continue
                pe = q0[0:3] - data.qpos[qa:qa+3]
                qw, qx, qy, qz = data.qpos[qa+3:qa+7]
                sgn = 1.0 if qw >= 0 else -1.0
                data.qfrc_applied[da:da+3] = fade * (80.0 * pe
                                                     - 8.0 * data.qvel[da:da+3])
                data.qfrc_applied[da+3:da+6] = fade * (
                    -6.0 * sgn * np.array([qx, qy, qz])
                    - 0.35 * data.qvel[da+3:da+6])
        elif stage == "t":
            # TEETER-TOTTER (Bill's ladder, rung 3): the fulcrum with one
            # axis locked — gy pinned level, gx free. Strictly easier than
            # stage B; here for the ladder's completeness and the show.
            data.qpos[yadr] = 0.0
            data.qvel[ydof] = 0.0
            if t < 2.0:
                data.qpos[xadr] = 0.0
                data.qvel[xdof] = 0.0
        elif stage == "b" and t < 2.0:
            # a steadying hand on the plate while he mounts: the spawn
            # settle used to slam the free plate to its stops before he
            # could possibly react
            data.qpos[xadr] = data.qpos[yadr] = 0.0
            data.qvel[xdof] = data.qvel[ydof] = 0.0
        elif stage == "b":
            # DISTURBANCE REJECTION (Bill: "say it gets disturbed, can he
            # bring it back?") — every 15 s an outside torque shove hits
            # the plate, direction rotating by the golden angle, magnitude
            # growing each round. Recovery = plate back under 1.5 deg held
            # for a second.
            data.qfrc_applied[xdof] = 0.0
            data.qfrc_applied[ydof] = 0.0
            k = int((t - 6.0) / 15.0) if t >= 6.0 else -1
            if k >= 0:
                t_k = 6.0 + k * 15.0
                if t_k <= t < t_k + 0.15:
                    mag = 0.08 + 0.045 * k
                    ang = k * 2.399
                    data.qfrc_applied[xdof] = mag * math.cos(ang)
                    data.qfrc_applied[ydof] = mag * math.sin(ang)
                    whack_mag[k] = round(mag, 3)
                    whack_ok_since = None
                elif k not in whack_rec:
                    p_t = (abs(float(data.qpos[xadr]))
                           + abs(float(data.qpos[yadr])))
                    if p_t < 0.026:
                        if whack_ok_since is None:
                            whack_ok_since = t
                        elif t - whack_ok_since > 1.0:
                            whack_rec[k] = round(whack_ok_since - (t_k + 0.15), 2)
                    else:
                        whack_ok_since = None
        elif stage == "static":
            # incline LADDER (Bill: "holding quite a high degree of incline
            # while maintaining a level chassis"): 3..18 deg in 3-deg steps,
            # 12 s each, slewed at ~2.3 deg/s; per-level chassis levelness
            lvl = min(int(t / 12.0), 5)
            tgt = math.radians(3.0 * (lvl + 1))
            amp_reached = max(amp_reached, tgt)
            cur = float(data.qpos[xadr])
            data.qpos[xadr] = cur + max(-8e-5, min(8e-5, tgt - cur))
            data.qvel[xdof] = 0.0
            if t - lvl * 12.0 > 4.0:            # settled part of the level
                _le = lvl_err.setdefault(lvl, [0.0, 0])
                _le[0] += abs(rp_of(data.body("base").xquat)[0]); _le[1] += 1
        elif stage == "step":
            # step response (Bill: "responsive, quick, and smooth"): plate
            # snaps 8 deg at t=2; clock chassis re-level (<1.5 deg held)
            tgt = math.radians(8.0) if t > 2.0 else 0.0
            cur = float(data.qpos[xadr])
            data.qpos[xadr] = cur + max(-2.4e-3, min(2.4e-3, tgt - cur))
            data.qvel[xdof] = 0.0
            if t > 2.0 and step_done is None:
                _att = abs(rp_of(data.body("base").xquat)[0])
                if _att < 0.026:
                    if step_ok_since is None:
                        step_ok_since = t
                    elif t - step_ok_since > 0.5:
                        step_done = round(step_ok_since - 2.0, 2)
                else:
                    step_ok_since = None
        data.ctrl[:] = drv.ctrl(model.opt.timestep)
        mujoco.mj_step(model, data)
        t += model.opt.timestep
        s += 1
        if renderer is not None and s % 16 == 0:
            cam = mujoco.MjvCamera(); cam.lookat[:] = data.body("base").xpos
            cc = cam_ctl or {}
            cam.distance = cc.get("distance", 0.8)
            cam.elevation = cc.get("elevation", -25)
            cam.azimuth = cc.get("azimuth", 140)
            renderer.update_scene(data, camera=cam)
            frame_cb(renderer.render())
            lag = (wall0 + t) - time.perf_counter()
            if lag > 0:
                time.sleep(lag)
        r_, p_ = rp_of(data.body("base").xquat)
        if t > 2.5 and (abs(r_) + abs(p_) > 0.7
                        or float(data.body("base").xpos[2]) < fall_z):
            # no falls while the steady hand is on (spawn settle dips)
            break
    res = dict(survived_s=round(t, 1),
               amp_reached_deg=round(math.degrees(amp_reached), 1),
               mean_att_deg=round(math.degrees(err_sum / max(1, err_n)), 2),
               fell=t < max_s, genes=bal)
    if stage == "static":
        res["ladder"] = {f"{3*(l+1)}deg": round(math.degrees(e[0]/max(1, e[1])), 2)
                         for l, e in sorted(lvl_err.items())}
        held = [3*(l+1) for l, e in lvl_err.items()
                if math.degrees(e[0]/max(1, e[1])) < 2.0]
        res["max_held_level_deg"] = max(held) if held else 0
    if stage == "step":
        res["response_s"] = step_done
    res["plate_mean_deg"] = round(math.degrees(plate_sum / max(1, plate_n)), 2)
    if whack_mag:
        res["whacks"] = [(whack_mag[k], whack_rec.get(k))
                         for k in sorted(whack_mag)]
    if verbose:
        print(json.dumps(res))
    return res


def _amp(t, stage):
    if stage != "a":
        return 0.0
    return math.radians(min(AMAX, A0 + RAMP * t))


def evolve(stage="a", gens=8, lam=6, seed=0):
    rng = np.random.default_rng(seed)
    best = dict(BAL)
    r = run_balance(best, stage, verbose=False)
    best_sc = _score(r)
    print(f"baseline (no servo): {r['survived_s']}s to "
          f"{r['amp_reached_deg']} deg  att {r['mean_att_deg']} deg", flush=True)
    for g in range(1, gens + 1):
        sig = 0.30 * (0.82 ** (g - 1))
        for _ in range(lam):
            kid = {}
            for k, (lo, hi) in BAL_BOUNDS.items():
                kid[k] = float(np.clip(best[k] + rng.normal(0, sig*(hi-lo)),
                                       lo, hi))
            r = run_balance(kid, stage, verbose=False)
            sc = _score(r)
            tag = (f"{r['survived_s']}s/{r['amp_reached_deg']}deg"
                   f"/att{r['mean_att_deg']}")
            print(f"  gen {g}  {sc:7.1f} ({tag})  {kid}", flush=True)
            if sc < best_sc:
                best_sc, best = sc, kid
        print(f"gen {g} BEST {best_sc:.1f}  {best}", flush=True)
        json.dump(dict(best=best, score=best_sc),
                  open(os.path.join(HERE, "out", f"balance_{stage}.json"), "w"))
    return best


def _score(r):
    # survival first (maximize -> negative), attitude error the tie-break
    # (on the ball, survival is nearly everything)
    return -r["survived_s"] + 8.0 * r["mean_att_deg"]


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", default="a", choices=["a", "b", "c", "t", "static", "step"])
    ap.add_argument("--watch", action="store_true")
    ap.add_argument("--evolve", action="store_true")
    ap.add_argument("--gens", type=int, default=8)
    ap.add_argument("--genes", default=None,
                    help="json file with genes (default out/balance_<stage>.json)")
    a = ap.parse_args()
    # until evolution has written something, demo the hand-verified servo —
    # never show Bill the zero-gain baseline as if it were the state of the
    # art ("he's not compensating, he's just standing there")
    genes = dict(kp=1.5, ki=0.8, leak=0.5, height=0.0)
    gp = a.genes or os.path.join(HERE, "out", f"balance_{a.stage}.json")
    if os.path.exists(gp):
        genes = json.load(open(gp))["best"]
        print(f"[BAL] genes from {os.path.basename(gp)}: {genes}")
    else:
        print(f"[BAL] no evolved genes yet — demoing hand-verified servo {genes}")
    if a.evolve:
        evolve(a.stage, gens=a.gens)
    elif a.watch:
        import importlib, matplotlib
        from watch_nav import (make_world_window, show_frame, SpectatorClosed)
        import matplotlib.pyplot as plt
        CAM = dict(azimuth=140.0, elevation=-25.0, distance=0.8)
        WW = make_world_window(CAM)
        tfig, tax = plt.subplots(num="Gary — balance telemetry", figsize=(6.6, 3.2))
        tax.set_xlabel("t (s)"); tax.set_ylabel("deg")
        pl_ln, = tax.plot([], [], color="#888", lw=1.2, label="platform |tilt|")
        bd_ln, = tax.plot([], [], color="#2d7fe8", lw=1.8, label="body |attitude|")
        am_ln, = tax.plot([], [], "--", color="#d43d3d", lw=1, label="difficulty amp")
        tax.legend(fontsize=8); tfig.tight_layout(); tfig.show()
        T = dict(t=[], plat=[], body=[], amp=[])

        def telem(d):
            for k in T:
                T[k].append(d[k if k != "t" else "t"])
            pl_ln.set_data(T["t"], T["plat"]); bd_ln.set_data(T["t"], T["body"])
            am_ln.set_data(T["t"], T["amp"])
            tax.relim(); tax.autoscale_view()
            tax.set_title(f"t={d['t']:.0f}s  amp {d['amp']:.1f} deg  "
                          f"body {d['body']:.1f} deg", fontsize=9)
            tfig.canvas.draw_idle(); tfig.canvas.flush_events()

        lap = 0
        import balance_gym as BG
        import gary_mujoco as GM
        while True:
            importlib.reload(GM)       # the WORLD first — reloading only BG
            # left it bound to a stale build_xml (TypeError mid-session)
            importlib.reload(BG)       # my edits land between laps, same as
            lap += 1                   # watch_nav (the jitter fix shipped
            # while a stale watcher kept showing the jitter)
            print(f"[BAL] lap {lap} (stage {a.stage})...")
            for k in T:
                T[k].clear()
            try:
                r = BG.run_balance(genes, a.stage, cam_ctl=CAM,
                                   frame_cb=lambda fr: show_frame(WW, fr),
                                   telem_cb=telem)
            except SpectatorClosed:
                print("[BAL] world window closed — done.")
                break
            print(f"[BAL] lap {lap}: {r['survived_s']}s, reached "
                  f"{r['amp_reached_deg']} deg, att {r['mean_att_deg']} deg")
            if os.path.exists(gp):
                genes = json.load(open(gp))["best"]   # pick up fresh evolutions
            time.sleep(2.0)
    else:
        run_balance(genes, a.stage)
