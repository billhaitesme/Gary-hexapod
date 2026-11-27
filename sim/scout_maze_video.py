"""SCOUT MODE in the maze (Bill, 2026-08-20): reflexes navigate corridors; the final
chamber's only exit is a LOW TUNNEL below the cruising lidar plane — the ring reads
360 deg of wall, so pure reflexes ping-pong forever. Gary must DECIDE he is confused
and invoke SCOUT v2 — CROUCH-AND-SCAN: drop low (his lidar plane sinks BELOW the
lintel), spin 360, and the tunnel is the one bearing the ring suddenly sees THROUGH.
Head there, stay low until clear. (Real Gary ducking under a coffee table gets this
exact view — fully hardware-true.)

All of it is real-hardware honest: 360 lidar ring, forward sonar swept by rotation,
IMU yaw for the spin, the graduated posture reflexes underneath.

Run:   uv run --with mujoco --with numpy python scout_maze.py            (headless)
       ... --view                                                        (live window)
       ... --video out/gary_scout_maze.mp4                               (mp4)
"""
import argparse, json, math, os, time
import numpy as np
import mujoco
import gary_mujoco as G

HERE = os.path.dirname(os.path.abspath(__file__))
FINISH_X = 2.42
SIM_MAX_S = 1500.0
CTRL_EVERY = 5
CAD, TRIP_LIFT = 493, 2.15

# honors-run postures (course_policy_sensor.json)
CRUISE = dict(speed=0.75, lift=0.022, height=0.020)   # tall: ring ABOVE the lintel (puzzle intact), knees below ring
DUCK   = dict(speed=0.60, lift=0.022, height=-0.020)
WALL_TRIG_MM   = 200      # ring front arc -> hazard turn
DUCK_RING_MM   = 250
SONAR_CLEAR_MM = 400
DUCK_TRAVEL_M  = 0.24
# confusion -> scout
CONFUSED_TURNS = 3
CONFUSED_WIN_S = 30.0
CONFUSED_DISP  = 0.35   # chamber is 0.7x0.9 — wandering inside still counts as confused
SCOUT_OMEGA    = 0.35


def yaw_of(q):
    return math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def run(view=False, video=None, verbose=True):
    model = mujoco.MjModel.from_xml_string(G.build_xml(maze=True))
    data = mujoco.MjData(model)
    data.qpos[2] = G.COXA_Z
    drv = G.GaitDriver("TRIPOD", 1.0, lift_cm=TRIP_LIFT, cadence_ms=CAD, speed_cap=1.0,
                       touchdown="match")
    heading = 0.0            # desired yaw; 0 = +y. forward = (-sin h, cos h)
    crouch_until = -1.0
    retreat_until = -1.0
    pivot_target = None
    turn_cool = 1.0
    wall_grace_until = 0.0
    turns = []               # timestamps of hazard turns (confusion window)
    anchor_pos = None
    anchor_t = 0.0
    scout = None             # dict when scouting
    cell_visits = {}         # coarse revisit map (odometry-honest confusion)
    cell_cur = None
    scout_cool = 0.0
    last_scout_t = -1.0
    traverse_until = -1.0
    from collections import deque
    hist = {k: deque(maxlen=7) for k in ("front", "left", "right", "sonar")}
    scout_done = False
    t = 0.0
    frames, renderer, viewer = [], None, None
    if video:
        renderer = mujoco.Renderer(model, 540, 960)
    if view:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data)
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = model.body("base").id
        viewer.cam.distance, viewer.cam.elevation = 0.8, -30
    wall0 = time.perf_counter()
    n = int(SIM_MAX_S / model.opt.timestep)
    result = dict(done=False, scout_used=False, reason="timeout")
    events = []
    for s in range(n):
        if s % CTRL_EVERY == 0:
            sd = data.sensordata
            sonar = 2.0 if sd[0] < 0 else float(sd[0])
            ring = [2.0 if v < 0 else float(v) for v in sd[1:25]]
            front_raw = min(ring[k % 24] for k in (-1, 0, 1))
            # +y forward, +x = HIS RIGHT. az = idx*15 with x=sin(az): idx 4-6 = right,
            # idx 18-20 = left (forward-biased so a passage beside the nose reads open).
            right_raw = min(ring[4:7]); left_raw = min(ring[18:21])
            # SELF-ECHO FILTER (2026-08-20: he was lidar-ing his own KNEES — swing tops
            # cross the ring plane ~0.2 s per stride). Blips are transient; walls
            # persist: rolling MAX over ~0.35 s erases the blips, keeps the walls.
            hist["front"].append(front_raw); hist["left"].append(left_raw)
            hist["right"].append(right_raw); hist["sonar"].append(sonar)
            front = max(hist["front"]); left = max(hist["left"]); right = max(hist["right"])
            sonar_f = max(hist["sonar"])
            q = data.body("base").xquat
            yawr = yaw_of(q)
            bx, by = data.body("base").xpos[0], data.body("base").xpos[1]
            pos = np.array([bx, by])

            # ---------------- SCOUT: THE GIMBAL SCAN (Bill's spec) ----------------
            # Body planted; commanded pitch+roll sweep a circle (NODSCAN pass 2:
            # "he uses himself as a gimbal"). The lidar plane rolls around the
            # room; each bearing gets seen dipped, level, and raised in ONE 16 s
            # revolution. A bearing whose DIPPED ray sees far past its LEVEL ray
            # = a low opening (tunnel). No yaw rotation needed at all.
            if scout is not None:
                A = math.radians(11.0)
                T_g = 16.0
                ph = 2.0 * math.pi * (t - scout["t0"]) / T_g
                drv.v = 0.0; drv.vx = 0.0; drv.omega = 0.0
                drv.lift, drv.height = 0.006, CRUISE["height"]   # scan with the SAME eyes that saw a wall
                drv.body_pitch = A * math.cos(ph)
                drv.body_roll  = A * math.sin(ph)
                if t - scout["t0"] > 1.0:                     # settle first
                    for b in range(24):
                        az = math.radians(15 * b)
                        elev = drv.body_pitch * math.cos(az) - drv.body_roll * math.sin(az)
                        rng = ring[b]
                        if elev < -0.7 * A:
                            scout["down"][b] = max(scout["down"][b], rng)
                        elif elev > 0.7 * A:
                            scout["up"][b] = max(scout["up"][b], rng)
                        elif abs(elev) < 0.25 * A:
                            scout["level"][b] = max(scout["level"][b], rng)
                if t - scout["t0"] > 1.0 + T_g:
                    drv.body_pitch = drv.body_roll = 0.0
                    # LOW-OPENING fingerprint: the RAISED ray smacks a nearby face
                    # (the lintel) while the DIPPED ray passes far beneath it.
                    best_b, best_gap = None, 0.0
                    for b in range(24):
                        gap = scout["down"][b] - scout["up"][b]
                        if scout["down"][b] > 0.55 and scout["up"][b] < 0.6 and gap > 0.30 and gap > best_gap:
                            best_gap, best_b = gap, b
                    tunnel_found = best_b is not None
                    if tunnel_found:
                        az = math.radians(15 * best_b)
                        heading = wrap(yawr - az)
                        events.append((round(t,1), f"GIMBAL: TUNNEL at body-bearing {15*best_b} deg (dipped sees {best_gap:.2f} m past the raised ray)"))
                    else:
                        def dest_visits(az, rt):
                            d = min(rt, 0.6)
                            h = wrap(yawr - az)
                            cx = round((bx - math.sin(h) * d) / 0.30)
                            cy = round((by + math.cos(h) * d) / 0.30)
                            return len(cell_visits.get((cx, cy), []))
                        cands = [(math.radians(15*b), scout["level"][b]) for b in range(24)
                                 if scout["level"][b] > 0.45]
                        if not cands:
                            cands = [(math.radians(15*b), scout["level"][b]) for b in range(24)]
                        az = min(cands, key=lambda x: (dest_visits(x[0], x[1]), -x[1]))[0]
                        heading = wrap(yawr - az)
                        events.append((round(t,1), f"GIMBAL: no tunnel -> least-visited frontier body-bearing {math.degrees(az):.0f} deg"))
                    scout = None
                    scout_done = True
                    scout_cool = t + 25.0
                    last_scout_t = t
                    crouch_until = (t + 0.65 / 0.012) if tunnel_found else -1.0
                    traverse_until = t + (55.0 if tunnel_found else 12.0)
                    turns.clear()
                    anchor_pos, anchor_t = pos.copy(), t
            else:
                # ---------------- RIGHT-HAND WALL FOLLOWING ----------------
                # The fifty-year-old maze classic: keep the right wall at RD,
                # turn left when blocked ahead, wrap right around openings.
                # Provably tours a simply-connected maze — including doors my
                # hand-rolled turn logic kept missing. Scout fires when boxed
                # or looping (revisit map).
                RD = 0.19
                crouching = t < crouch_until
                if traverse_until > t:
                    # post-scout traverse: hold the verdict bearing
                    yaw_err = wrap(yawr - heading)
                    drv.v = G.WALK_SPEED_CM_S * 0.01 * (DUCK["speed"] if crouching else CRUISE["speed"])                             * (1.0 if abs(yaw_err) < 1.0 else 0.1)
                    drv.omega = float(np.clip(-1.5 * yaw_err, -0.4, 0.4))
                    drv.vx = 0.0
                else:
                    blocked = front < 0.20
                    door_right = right > 0.50
                    if blocked:
                        drv.v = 0.0
                        drv.omega = 0.35          # rotate LEFT until clear
                    elif door_right:
                        drv.v = G.WALK_SPEED_CM_S * 0.01 * 0.55
                        drv.omega = -0.26         # arc RIGHT around the opening
                    else:
                        drv.v = G.WALK_SPEED_CM_S * 0.01 * (DUCK["speed"] if crouching else CRUISE["speed"])
                        drv.omega = float(np.clip(-1.0 * (right - RD), -0.28, 0.28))
                    drv.vx = 0.0
                pk = DUCK if crouching else CRUISE
                drv.lift = pk["lift"]
                drv.height = pk["height"]

                # visit map + confusion triggers
                cell = (round(bx / 0.30), round(by / 0.30))
                if cell != cell_cur:
                    cell_cur = cell
                    hist_v = cell_visits.setdefault(cell, [])
                    if not hist_v or t - hist_v[-1] > 8.0:
                        hist_v.append(t)
                revisits = max((len([x for x in v if x > last_scout_t])
                                for v in cell_visits.values()), default=0)
                boxed = front < 0.20 and left < 0.25 and right < 0.25
                if (boxed or revisits >= 4) and scout is None and t > scout_cool:
                    scout = dict(t0=t, down=[0.0]*24, level=[0.0]*24, up=[0.0]*24)
                    result["scout_used"] = True
                    events.append((round(t,1), f"{'boxed' if boxed else 'looping'} at ({bx:+.2f},{by:+.2f}) -> SCOUT MODE"))
        data.ctrl[:] = drv.ctrl(model.opt.timestep)
        mujoco.mj_step(model, data)
        t += model.opt.timestep
        if viewer is not None:
            if not viewer.is_running():
                break
            viewer.sync()
            lag = (wall0 + t) - time.perf_counter()
            if lag > 0:
                time.sleep(lag)
        if renderer is not None and s % 16 == 0 and t > 500.0:
            cam = mujoco.MjvCamera(); cam.lookat[:] = data.body("base").xpos
            cam.distance, cam.elevation, cam.azimuth = 0.8, -35, 150
            renderer.update_scene(data, camera=cam)
            frames.append(renderer.render().copy())
        if data.body("base").xpos[0] >= FINISH_X:
            result.update(done=True, reason="finished")
            break
        if data.body("base").xpos[2] < 0.02:
            result.update(reason="fell")
            break
    if viewer is not None:
        viewer.close()
    result.update(time_s=round(t, 1),
                  pos=[round(float(data.body("base").xpos[0]), 2),
                       round(float(data.body("base").xpos[1]), 2)])
    if verbose:
        for ev in events:
            print(f"  t={ev[0]:6.1f}  {ev[1]}")
        print(json.dumps(result))
    if video and frames:
        import imageio
        imageio.mimwrite(video, frames, fps=30, quality=8)
        result["video"] = video
    return result

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--video", default=None)
    a = ap.parse_args()
    run(view=a.view, video=a.video)
