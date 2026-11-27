"""Stage-3c: SENSOR-driven course policy — Gary decides from what he can SENSE.

Unlike train_course.py (zone-indexed by position = cheating), this policy only
sees what real Gary can sense:
    sonar     ultrasonic eyes, 10 deg down-tilt; floor baseline ~0.215 m,
              a low bar breaks the baseline early  -> climb trigger
    nod_min   nod-scan sample rays (+12 deg front sector) -> catches the bridge
              deck                                  -> crouch trigger
    pitch     IMU — the see-saw and stairs tilt him -> stability slowdown

The policy is 14 learned numbers (CMA-ES):
    cruise:  speed, lift, height, gait
    climb:   sonar trigger dist, speed, lift, height, persistence seconds
    crouch:  nod trigger dist, speed, height
    avoid_steer (side-step bias while a trigger is active; ~0 = go over/under,
                 large = go around)
    tilt_slow (speed multiplier while |pitch| > 8 deg)
Every threshold and response is LEARNED, not scripted. The same rule shapes are
firmware-portable: sonar + nod-scan min + IMU pitch all exist on the real robot.

Course v2 (gary_mujoco obstacles): bar 12mm (y.35), bar 22mm (y.70), SOLID bridge
119mm clearance (y1.15), see-saw (y1.85), stairs 4x15mm (y2.43-2.61), plateau, then a
walled DEAD END (y3.0) -> he must U-TURN and run the whole course backwards.
Finish = back at y<0.05 after touching the far end. Contacting the LiDAR mast on
anything = heavy penalty (protect the sensor — learned, not scripted).

Run:    uv run --with mujoco --with numpy --with cma python train_course_sensor.py --gens 16 --pop 10
Replay: uv run --with mujoco --with numpy python train_course_sensor.py --replay          (live)
        ... --replay --video out/gary_course_sensor.mp4                                    (mp4)
"""
import argparse, json, math, os, time
import numpy as np
import mujoco
import gary_mujoco as G

HERE = os.path.dirname(os.path.abspath(__file__))
BEST_PATH = os.path.join(HERE, "out", "course_policy_sensor.json")
FAR_Y = 2.70   # 'made it to the far end' = anywhere on the plateau
HOME_Y = 0.05
SIM_MAX_S = 150.0
CTRL_EVERY = 5
TRAINED = {"TRIPOD": (493, 2.15), "WAVE": (911, 1.09), "RIPPLE": (416, 0.76)}
GAITS = list(TRAINED)
RMAX = 2.0   # rangefinder no-hit -> this

KEYS = ["cr_speed", "cr_lift", "cr_height", "cr_gait",
        "cl_trig", "cl_speed", "cl_lift", "cl_height", "cl_hold",
        "du_trig", "du_speed", "du_height",
        "avoid_steer", "tilt_slow", "wall_trig", "cl_gait", "jolt_trig", "jolt_hold"]
LO = np.array([0.15, 0.006, -0.020, 0.0,  0.045, 0.10, 0.010, -0.005, 0.5,  0.10, 0.10, -0.022, -1.0, 0.30, 0.10, 0.0,  0.5, 0.2])
HI = np.array([1.20, 0.028,  0.040, 2.99, 0.15, 0.90, 0.030,  0.045, 3.0,  0.60, 0.90, -0.006,  1.0, 1.00, 0.45, 2.99, 6.0, 1.5])
# cl_trig is now a SONAR SHORTFALL (baseline minus reading, m) — height-invariant.

def decode(x):
    x = np.clip(np.asarray(x, dtype=float), 0, 1)
    return dict(zip(KEYS, LO + x * (HI - LO)))

def rf(v):
    return RMAX if v < 0 else float(v)

def rollout(params, view=False, video=None, verbose=False):
    p = decode(params)
    model = mujoco.MjModel.from_xml_string(G.build_xml(obstacles=True))
    data = mujoco.MjData(model)
    data.qpos[2] = G.COXA_Z
    knee_ids = [3*i+2 for i in range(6)]
    gi = min(2, int(p["cr_gait"]))
    cad, _ = TRAINED[GAITS[gi]]
    drv = G.GaitDriver(GAITS[gi], 1.0, lift_cm=2.0, cadence_ms=cad, speed_cap=1.0,
                       touchdown="match")
    cur_gait = gi
    def set_gait(gidx):
        nonlocal drv, cur_gait
        if gidx == cur_gait:
            return
        cad2, _ = TRAINED[GAITS[gidx]]
        nd = G.GaitDriver(GAITS[gidx], 1.0, lift_cm=2.0, cadence_ms=cad2, speed_cap=1.0,
                          touchdown="match")
        # phase-continuous handoff: carry the CYCLE FRACTION, not the raw clock
        # (a different cadence reinterprets t -> legs would teleport mid-stride)
        frac = (drv.t / drv.T) % 1.0
        nd.t = frac * nd.T
        nd.height, nd.lift = drv.height, drv.lift
        drv, cur_gait = nd, gidx
    sat = steps = 0
    t = 0.0
    climb_until = -1.0
    crouch_until = -1.0
    jolt_until = -1.0
    prev_pitch = prev_roll = 0.0
    jolt_ema = 0.0
    last_prog, last_prog_t = 0.0, 0.0
    mode = "cruise"
    lidar_gid = model.geom("lidar").id
    lidar_hits = 0
    heading = 0.0            # desired yaw: 0 = outbound (+y), pi = homebound
    premature_turn = False   # Bill, 2026-08-19: a premature homeward turn ends the
                             # episode immediately — walking back teaches nothing
    retreat_until = -1.0     # backing away from a hazard (Bill: "he should just back up")
    retreat_reason = "wall"
    pending_flip = False
    turn_dir = 1.0           # pivot toward the open side (chosen at flip time)
    reached_far = False
    by_max = 0.0
    turn_cooldown_until = -1.0
    frames = []
    renderer = mujoco.Renderer(model, 540, 960) if video else None
    viewer = None
    if view:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data)
        # camera follows Gary (Bill: room-fixed cam loses him at the track's far end);
        # mouse orbit/zoom still work, just relative to him.
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = model.body("base").id
        viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 0.7, -20, 140
    wall0 = time.perf_counter()
    n = int(SIM_MAX_S / model.opt.timestep)
    result = None
    for s in range(n):
        if s % CTRL_EVERY == 0:
            sd = data.sensordata
            sonar = rf(sd[0])
            nod_min = min(rf(v) for v in sd[25:30])
            high_min = min(rf(v) for v in sd[30:33])   # +35 deg rays: pass over the thin
                                                        # deck, hit TALL things = walls
            q = data.body("base").xquat
            # nose pitch = elevation of the body-forward (+y) axis: (R*yhat)_z.
            # (The textbook asin(2(q0q2-q3q1)) measures rotation about +y — that is
            # BANKING in this body frame, not nose attitude. Caught 2026-08-19.)
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2*(q[2]*q[3] + q[0]*q[1])))))
            bank  = math.degrees(math.asin(max(-1.0, min(1.0, 2*(q[0]*q[2] - q[3]*q[1])))))
            yawr = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
            bx = data.body("base").xpos[0]
            ring = [rf(v) for v in sd[1:25]]
            # body-forward IS travel-forward: after the U-turn the body physically
            # rotates 180, so the same forward rays keep looking where he walks.
            # (fidx=12 flipping was a bug — it made the whole return leg blind.)
            fidx = 0
            front_min = min(ring[(fidx + k) % 24] for k in (-2, -1, 0, 1, 2))
            # side clearances (for corridor centering under the bridge):
            # ring az: index*15deg with +x at 90deg -> right = fidx+4..8, left = fidx+16..20
            right_min = min(ring[(fidx + k) % 24] for k in (4, 5, 6, 7, 8))
            left_min  = min(ring[(fidx + k) % 24] for k in (16, 17, 18, 19, 20))
            rear_min  = min(ring[(fidx + k) % 24] for k in (10, 11, 12, 13, 14))
            # Baseline-relative sonar: the down-tilted ray hits open floor at a
            # distance set by the sonar's height — which shifts with commanded
            # body height. Gary knows his own height, so the ONBOARD-computable
            # quantity is the SHORTFALL vs that baseline. Height-invariant.
            sonar_z = 0.040 + drv.height              # sonar origin above ground (commanded)
            # TILT COMPENSATION (Bill, 2026-08-19: false cliffs while cresting bars/
            # stairs): the ray's true down-angle = 10 deg mount tilt MINUS body pitch
            # (nose-up flattens it, nose-down steepens it). Judge shortfall/overshoot
            # against the tilt-corrected baseline; if the ray is nearly horizontal
            # (< 3 deg) it can't judge the floor at all — skip both triggers.
            eff_deg = 10.0 - pitch                    # pitch > 0 = nose-up (IMU)
            if eff_deg > 3.0:
                baseline = sonar_z / math.tan(math.radians(eff_deg))
                shortfall = max(0.0, baseline - sonar)
                overshoot_ok = True
            else:
                baseline = sonar
                shortfall = 0.0
                overshoot_ok = False
            # CLIFF DETECTION (Bill, 2026-08-19: he kept walking off the plateau):
            # the OTHER half of the sonar signal — a return LONGER than the floor
            # baseline means the floor ahead is missing/lower. A 15 mm stair-down
            # only overshoots ~8 cm; the plateau's 60 mm drop overshoots ~34 cm.
            # 0.15 m separates "stairs, fine" from "cliff, do not proceed".
            # (Same trick works on the real sonar; VL53L0X corner ToFs are the
            # hardware upgrade — see notes/STATUS.)
            overshoot = max(0.0, sonar - baseline) if overshoot_ok else 0.0
            # CLIFF verdicts only when the geometry can actually support one
            # (2026-08-19 collapse: cresting bar1 at eff~3.5deg ballooned the
            # baseline to 0.65m, ray grazed over bar2 -> false cliff -> fail-fast
            # executed every good climber; population converged on stalling):
            #   - ray steep enough (eff >= 7deg -> baseline <= ~0.33m lookahead)
            #   - never while climbing (standing on the obstacle = height unmodeled)
            cliff = (overshoot > 0.15 and eff_deg >= 7.0 and baseline < 0.33
                     and t >= climb_until and t >= crouch_until and abs(pitch) < 20.0)
            # Sensor fusion (bridge-vs-wall, Bill 2026-08-19): mast-height return
            # + small shortfall (floor clear below) = overhead -> duck under.
            # Mast-height return + BIG shortfall = wall -> U-turn.
            # nod rays keep the crouch held while actually under the deck.
            # Fusion v4 (priority ladder, hardened by two failure trades):
            #  1. UNDER something (nod rays close overhead) -> crouch, and wall
            #     logic is suppressed entirely (under the deck the +35 deg rays
            #     hit its underside and would false-alarm "wall").
            #  2. APPROACHING a thin deck: low rays blocked, steep rays pass
            #     clean over -> crouch early (knees rise ~13 mm in tall swing —
            #     they hit the bridge lip if he waits).
            #  3. WALL: BOTH low and steep rays blocked -> U-turn. If pressed
            #     against a wall while crouched, the unstick reflex backs him
            #     off until nod clears, then this fires.
            # "under a roof" = nod blocked but ring CLEAR (rays pass beneath the deck).
            # nod + ring both short = that's a WALL FACE, not a roof — let wall logic
            # act (found crouch-locked kneeling at the enclosure wall, 2026-08-19).
            under = (nod_min < 0.18) and (front_min > 0.30)
            approach_deck = (front_min < p["du_trig"]) and (high_min > 0.45)
            if under or approach_deck:
                # DISTANCE-based release (Bill: head bumps exiting the tunnel both
                # ways): nod rays look ~10-30 cm AHEAD, so they clear while the mast
                # is still under the deck. Stay low until ~24 cm of travel past the
                # last overhead sighting (speed-scaled; real Gary: leg odometry).
                v_now = max(abs(drv.v), 0.004)
                crouch_until = t + min(0.24 / v_now, 14.0)
            crouching = t < crouch_until
            yaw_err_now = math.atan2(math.sin(yawr - heading), math.cos(yawr - heading))
            pivoting = abs(yaw_err_now) > 0.5
            # a WALL spans the forward arc; a narrow post blocks only 1-2 rays
            front_hits = sum(1 for k in (-2, -1, 0, 1, 2)
                             if ring[(fidx + k) % 24] < p["wall_trig"])
            wall = (not crouching) and (front_hits >= 4) and (high_min < 0.45)
            if cliff:
                wall = True          # a cliff is treated like a wall: leave
            # HAZARD RESPONSE (Bill, 2026-08-19): don't pirouette on the cliff edge —
            # BACK UP first (~1 body length, sensors still facing the hazard), THEN
            # turn around with margin. Latched: the hazard signal vanishes as soon
            # as he backs off, so commit to the full retreat.
            if wall and t > turn_cooldown_until and t > 0.3:
                retreat_reason = "cliff" if cliff else "wall"
                retreat_until = t + (0.7 if cliff else 1.6)   # cliff: short pull-back; wall: until clearance
                pending_flip = True
                turn_cooldown_until = t + 14.0
            # ADAPTIVE RETREAT (Bill: "1 body length might be a bit much"):
            # stop backing as soon as there is pivot clearance — or immediately
            # if the rear rays say there is nothing to back into (tight spot:
            # skip the retreat, pivot carefully where he stands).
            if pending_flip and t < retreat_until:
                enough = (retreat_reason == "wall" and front_min > 0.16) or rear_min < 0.09
                if enough:
                    retreat_until = t
            if pending_flip and t >= retreat_until:
                heading = math.pi if heading == 0.0 else 0.0
                turn_dir = 1.0 if left_min > right_min else -1.0   # spin toward the open side
                pending_flip = False
                if heading != 0.0 and not reached_far:
                    premature_turn = True   # turned for home without reaching the far end
            if shortfall > p["cl_trig"] and t > 0.3:
                climb_until = t + p["cl_hold"]
            # CLIMB PERSISTENCE (Bill, 2026-08-19: "stuck on the 2nd bar, not
            # changing to climb"): once the front is up on the bar the sonar sees
            # clean over it and the hold expires MID-CROSSING — belly beaches.
            # The IMU knows better: body inclined = still on the obstacle, hold
            # the climb posture until level again. (Firmware-portable: same IMU.)
            if abs(pitch) > 5.0 and not crouching and t > 0.3:
                climb_until = max(climb_until, t + 1.0)
            climbing = (t < climb_until) and not crouching
            if crouching:
                mode = "crouch"
                speed, lift, height = p["du_speed"], p["cr_lift"], p["du_height"]
                if not pivoting: set_gait(min(2, int(p["cr_gait"])))
            elif climbing:
                mode = "climb"
                speed, lift, height = p["cl_speed"], p["cl_lift"], p["cl_height"]
                if not pivoting: set_gait(min(2, int(p["cl_gait"])))    # climb can use its OWN gait (Bill)
            else:
                mode = "cruise"
                speed, lift, height = p["cr_speed"], p["cr_lift"], p["cr_height"]
                if not pivoting: set_gait(min(2, int(p["cr_gait"])))
            if abs(pitch) > 8.0:
                speed *= p["tilt_slow"]
            # IMU ROUGH-MODE OVERLAY (winner of the sensor experiment): jolt spike ->
            # temporary high-step + slowdown. Not while crouched (no headroom).
            jolt = abs(pitch - prev_pitch)
            prev_pitch = pitch
            jolt_ema = 0.7 * jolt_ema + 0.3 * (10.0 * jolt)
            if jolt_ema > p["jolt_trig"] and mode == "cruise" and t > 0.3:
                jolt_until = t + p["jolt_hold"]
            if t < jolt_until and mode == "cruise":
                # cruise-only: a deliberate climb/crouch already manages its posture —
                # this overlay is for UNEXPECTED roughness on open ground.
                lift = max(lift, 0.024)
                speed *= 0.7
            # UNSTICK REFLEX (fixed, not learned): no progress for 3 s -> alternate
            # 1.5 s "push-up" bursts (max height + max lift, keep walking) with
            # 1.5 s reverse. While crouching, invert: go LOWER instead of taller
            # (raising the mast under the deck would smash the LiDAR).
            if t < retreat_until:
                speed = -0.55        # hazard retreat overrides mode speed
            stalled = (t - last_prog_t)
            yaw_err_pre = math.atan2(math.sin(yawr - heading), math.cos(yawr - heading))
            if stalled > 3.0 and abs(yaw_err_pre) < 0.5 and t >= retreat_until:   # not mid-U-turn/retreat
                phase = int((stalled - 3.0) / 1.5) % 2
                on_obstacle = abs(pitch) > 5.0
                if phase == 0 or on_obstacle:
                    # push-up surge. NEVER reverse while inclined (Bill, 2026-08-19:
                    # beached on bar2 — backing up mid-straddle drags the rear legs
                    # into the bar; when on an obstacle, the only way out is over).
                    height = -0.022 if crouching else 0.045
                    lift = 0.028
                    speed = 0.65 if on_obstacle else 0.5
                else:
                    speed = -0.45
            yaw_err = math.atan2(math.sin(yawr - heading), math.cos(yawr - heading))
            if abs(yaw_err) > 0.5:
                # PIVOTING (Bill, 2026-08-19: pivot trouble): hold a stable stance —
                # no gait swaps, no mode churn; sonar sweeping walls mid-spin was
                # flickering climb mode and teleporting the gait phase.
                speed = min(speed, 0.3)
                lift = max(lift, 0.015)
                set_gait(cur_gait)     # explicit no-op: freeze gait while turning
            fwd_sign = 1.0 if abs(yaw_err) < 1.2 else 0.15   # creep while still turning
            drv.v = G.WALK_SPEED_CM_S * 0.01 * speed * fwd_sign
            in_lane = (left_min < 0.55) and (right_min < 0.55) and abs(bank) < 5.0
            # bank gate (Bill, 2026-08-19: side-stepping mid-see-saw): when rolled,
            # the low-side ring rays hit the tilted deck/floor and fake a side wall —
            # he was centering himself against reflections of the ground.
            if (crouching or in_lane) and abs(bank) < 5.0:
                # CORRIDOR / LANE CENTERING: whenever both sides return (bridge posts,
                # or the course rails), strafe toward the bigger gap. Pure sensing.
                gap = min(right_min, 0.5) - min(left_min, 0.5)   # body frame = travel frame
                drv.vx = float(np.clip(0.25 * gap, -0.02, 0.02))
            else:
                x_aim = p["avoid_steer"] * 0.15 if mode == "climb" else 0.0
                steer = (x_aim - bx)
                if heading != 0.0:
                    steer = -steer          # world-x error maps to mirrored body-x when turned
                drv.vx = float(np.clip(2.5 * steer, -0.03, 0.03))
            if abs(yaw_err) > 2.6:
                drv.omega = 0.4 * turn_dir            # break the 180-degree ambiguity toward open space
            else:
                drv.omega = float(np.clip(-1.5 * yaw_err, -0.4, 0.4))
            drv.lift = float(lift)
            drv.height = float(height)
        data.ctrl[:] = drv.ctrl(model.opt.timestep)
        mujoco.mj_step(model, data)
        t += model.opt.timestep
        steps += 1
        f = np.abs(data.actuator_force[knee_ids])
        sat += int((f > G.SERVO_TORQUE * 0.95).any())
        for ci in range(data.ncon):
            c = data.contact[ci]
            if c.geom1 == lidar_gid or c.geom2 == lidar_gid:
                lidar_hits += 1
                break
        by = data.body("base").xpos[1]
        bz = data.body("base").xpos[2]
        roll = math.degrees(math.atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]**2+q[2]**2))) if False else 0
        fell = bz < 0.020 or abs(pitch) > 50
        by_max = max(by_max, by)
        if by_max >= FAR_Y:
            reached_far = True
        # outbound: credit = y. After the U-turn: credit = how deep he got plus
        # how far back he has come (behavior-keyed, not a hairline position gate).
        prog_now = by if heading == 0.0 else (by_max + (by_max - max(by, HOME_Y)))
        if prog_now > last_prog + 0.01:
            last_prog, last_prog_t = prog_now, t
        stuck = (t - last_prog_t) > 20.0   # room for the U-turn (~10 s) and unstick attempts
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
        home = heading != 0.0 and by <= HOME_Y and reached_far   # true round trip
        if home or fell or stuck or premature_turn:
            result = dict(done=bool(home), full_far=bool(reached_far), fell=bool(fell),
                          stuck=bool(stuck), premature_turn=bool(premature_turn))
            break
    if viewer is not None:
        viewer.close()
    if result is None:
        result = dict(done=False, fell=False, stuck=False)
    prog = float(last_prog)
    sat_pct = 100.0 * sat / max(steps, 1)
    lidar_hit_s = lidar_hits * model.opt.timestep
    score = (-100.0 * prog + 8.0 * (t / 60.0) + 0.10 * sat_pct
             + (40.0 if result["fell"] else 0.0) + 25.0 * min(lidar_hit_s, 4.0)
             + (15.0 if result.get("premature_turn") else 0.0))
    result.update(progress_y=round(prog, 3), time_s=round(t, 1),
                  knee_sat_pct=round(sat_pct, 1), lidar_contact_s=round(lidar_hit_s, 2),
                  reached_far=bool(reached_far), score=round(score, 2), last_mode=mode)
    if verbose:
        print(result, flush=True)
    if video and frames:
        import imageio
        imageio.mimwrite(video, frames, fps=30, quality=8)
        result["video"] = video
    return score, result

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gens", type=int, default=16)
    ap.add_argument("--pop", type=int, default=10)
    ap.add_argument("--replay", action="store_true")
    ap.add_argument("--video", default=None)
    a = ap.parse_args()
    if a.replay:
        best = json.load(open(BEST_PATH))
        _, r = rollout(best["params"], view=not a.video, video=a.video, verbose=True)
        print(json.dumps(r, indent=1))
        return
    import cma
    # warm start from the one-way graduate if available
    x0 = [0.5] * 18
    try:
        prev = json.load(open(BEST_PATH))["params"]
        if 14 <= len(prev) <= 17:
            x0 = list(prev) + [0.5] * (18 - len(prev))
        elif len(prev) == 18:
            x0 = list(prev)
        print("warm start from previous best")
    except Exception:
        pass
    es = cma.CMAEvolutionStrategy(x0, 0.22, {"popsize": a.pop, "bounds": [0, 1],
                                             "seed": 5, "verbose": -9})
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
        # checkpoint best-so-far each generation (live spectators reload this)
        os.makedirs(os.path.dirname(BEST_PATH), exist_ok=True)
        json.dump(dict(params=best[1], metrics=best[2], gen=gen + 1),
                  open(BEST_PATH, "w"), indent=1)
        b = best[2]
        print(f"gen {gen+1}/{a.gens} best={best[0]:.1f} prog={b['progress_y']}m "
              f"t={b['time_s']}s sat={b['knee_sat_pct']}% done={b['done']}", flush=True)
    os.makedirs(os.path.dirname(BEST_PATH), exist_ok=True)
    p = decode(best[1])
    readable = {k: (round(float(v), 3) if not k.endswith("gait") else GAITS[min(2, int(v))])
                for k, v in p.items()}
    json.dump(dict(params=best[1], metrics=best[2], policy=readable),
              open(BEST_PATH, "w"), indent=1)
    print("WROTE", BEST_PATH)
    print(json.dumps(dict(metrics=best[2], policy=readable), indent=1))

if __name__ == "__main__":
    main()
