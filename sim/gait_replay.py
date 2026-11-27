"""Stage-1 gait sim: replay GARY_v3's custom wave/ripple scheduler in pure math,
no hardware, and measure the things that kill knee servos.

What it does
------------
Ports the scheduler from GARY_v3.ino (`run_custom_gait_scheduler`: phase offsets,
stance fraction beta, linear stance / cosine-swing / sin^2 lift, per-gait speed caps,
the 2026-08-18 slew limiter) and the vendor amplitude convention (amp = v*T/2), then
runs each gait across a sweep of commanded speeds and reports per leg:

  * knee + femur joint-angle envelopes (deg, physical, from CAD geometry)
  * peak joint angular rates (deg/s) vs what the servo can actually do
  * touchdown scoot: foot world-velocity at the moment of touchdown (should be ~0;
    whatever is left is dragged through the servo as stall current)
  * reachability: IK failures / clamp hits per cycle

Geometry: cad_tools ground truth (gary_geometry.h) — L1 29.20, L2 37.83, L3 74.35 mm,
neutral foot 71.07 mm out / 46.05 mm below the femur axis (the CAD standing pose).

Honest limits of stage 1
------------------------
* Kinematic only: no gravity, no torque, no servo lag. It finds *commanded* abuse
  (rate spikes, unreachable targets, touchdown velocity mismatch), not load effects.
* The vendor lib's c=0 duty convention is unverified on hardware, so angles are
  reported as PHYSICAL knee interior angles, not duties. Bench maps one to the other.
* Stage 2 (MuJoCo twin built from the CAD meshes) adds torque/contact and is where
  "train it virtually" happens — see sim/README.md.

Run:  uv run --with numpy python gait_replay.py
"""
import numpy as np, math, json, os

# ---- geometry (mm, from gary_geometry.h / CAD) --------------------------------
L1, L2, L3 = 29.20, 37.83, 74.35
NEUTRAL_OUT, NEUTRAL_DOWN = 71.07, 46.05      # foot rel. coxa axis / femur axis (CAD pose)
KNEE_CAD_POSE_DEG = 56.7                      # knee interior angle in the CAD pose
SERVO_DEG_PER_S = 60.0 / 0.12                 # ~typical micro-servo 0.12 s/60deg -> 500 deg/s
KNEE_HEADROOM_DEG = 50.0                      # c=0 sits 50 deg from the mechanical stop

# ---- scheduler constants (verbatim from GARY_v3.ino) ---------------------------
GAITS = ["TRIPOD", "WAVE", "RIPPLE"]
PHASE = [[0, .5, 0, .5, 0, .5],
         [4/6, 2/6, 0/6, 1/6, 3/6, 5/6],
         [0/3, 1/3, 2/3, 0/3, 1/3, 2/3]]
BETA  = [0.5, 5/6, 2/3]
SPEED_CAP  = [1.0, 0.15, 0.30]
MIN_CADENCE_MS = [400, 1200, 800]
LIFT_CM = 2.5
WALK_SPEED = 3.5           # gary_config.h WALK_SPEED (vendor units ~ cm/s at stick=1)
SLEW_MAX_CM_S, SLEW_CAP_CM = 40.0, 0.8   # the 2026-08-18 limiter
DT_MS = 1                  # LOOP_DELAY_MS 1

def leg_ik(out_mm, down_mm):
    """Planar 2-link IK in the leg's radial plane. Returns (femur_pitch_deg,
    knee_interior_deg) or None if unreachable. out = horizontal distance from
    COXA axis, down = below femur axis."""
    r = out_mm - L1
    D2 = r * r + down_mm * down_mm
    D = math.sqrt(D2)
    if D > L2 + L3 - 1e-6 or D < abs(L2 - L3) + 1e-6:
        return None
    knee = math.degrees(math.acos((L2 * L2 + L3 * L3 - D2) / (2 * L2 * L3)))
    a = math.atan2(-down_mm, r) + math.acos((L2 * L2 + D2 - L3 * L3) / (2 * L2 * D))
    return math.degrees(a), knee

def run(gait, stick, seconds=6.0):
    """Replay one gait at one stick deflection. Returns metrics dict."""
    g = GAITS.index(gait)
    v_cmd = WALK_SPEED * stick * SPEED_CAP[g]          # cm/s forward
    T_ms = max(MIN_CADENCE_MS[g], 400)                  # move_time per cycle segment
    T = T_ms / 1000.0
    amp = v_cmd * T / 2.0                               # vendor: amplitude = v*T/2 (cm, half-stroke)
    beta = BETA[g]
    dt = DT_MS / 1000.0
    n = int(seconds / dt)
    slew_prev = np.zeros((6, 3))
    init = False
    knee_min = np.full(6, 1e9); knee_max = np.full(6, -1e9)
    fem_min = np.full(6, 1e9);  fem_max = np.full(6, -1e9)
    knee_rate_max = np.zeros(6); fem_rate_max = np.zeros(6)
    prev_ang = {}
    ik_fail = 0
    scoot = []          # foot world-velocity magnitude at touchdown (cm/s)
    prev_phase = np.zeros(6)
    prev_foot_x = np.zeros(6)
    cycle_t = 0.0
    for step in range(n):
        cycle_t += dt / T
        cycle_t %= 1.0
        for i in range(6):
            ph = (cycle_t - PHASE[g][i]) % 1.0
            # scheduler trajectory, x = fore-aft in cm relative to neutral
            if ph < beta:
                k = 1.0 - 2.0 * ph / beta
                x = amp * k; z = 0.0
            else:
                st = (ph - beta) / (1.0 - beta)
                k = -math.cos(math.pi * st)
                x = amp * k
                s = math.sin(math.pi * st)
                z = LIFT_CM * s * s
            tgt = np.array([x, 0.0, z])
            # slew limiter (2026-08-18 firmware state)
            if not init:
                slew_prev[i] = tgt
            step_cap = min(SLEW_MAX_CM_S * dt, SLEW_CAP_CM)
            d = np.clip(tgt - slew_prev[i], -step_cap, step_cap)
            slew_prev[i] = slew_prev[i] + d
            fx, fy, fz = slew_prev[i]
            # foot position in the leg radial plane (mm): x moves along the body's
            # travel axis; for the worst-case leg that is fully radial.
            out = NEUTRAL_OUT + fx * 10.0
            down = NEUTRAL_DOWN - fz * 10.0
            ik = leg_ik(out, down)
            if ik is None:
                ik_fail += 1
                continue
            fem, knee = ik
            knee_min[i] = min(knee_min[i], knee); knee_max[i] = max(knee_max[i], knee)
            fem_min[i] = min(fem_min[i], fem);   fem_max[i] = max(fem_max[i], fem)
            if (i, "k") in prev_ang:
                knee_rate_max[i] = max(knee_rate_max[i], abs(knee - prev_ang[(i, "k")]) / dt)
                fem_rate_max[i] = max(fem_rate_max[i], abs(fem - prev_ang[(i, "f")]) / dt)
            prev_ang[(i, "k")] = knee; prev_ang[(i, "f")] = fem
            # touchdown detection: swing -> stance transition
            if prev_phase[i] >= beta and ph < beta and step > 100:
                # foot world velocity = body velocity + foot body-frame velocity
                foot_v_body = (slew_prev[i][0] - prev_foot_x[i]) / dt   # cm/s
                scoot.append(abs(v_cmd + foot_v_body))
            prev_phase[i] = ph
            prev_foot_x[i] = slew_prev[i][0]
        init = True
    return dict(gait=gait, stick=stick, v_cmd_cm_s=round(v_cmd, 2), cadence_ms=T_ms,
                amp_cm=round(amp, 2),
                knee_deg=[round(float(knee_min.min()), 1), round(float(knee_max.max()), 1)],
                femur_deg=[round(float(fem_min.min()), 1), round(float(fem_max.max()), 1)],
                knee_rate_max=round(float(knee_rate_max.max()), 0),
                femur_rate_max=round(float(fem_rate_max.max()), 0),
                servo_rate_limit=round(SERVO_DEG_PER_S, 0),
                touchdown_scoot_cm_s=round(float(np.mean(scoot)), 2) if scoot else None,
                ik_failures=ik_fail)

if __name__ == "__main__":
    print(f"geometry: L1={L1} L2={L2} L3={L3} mm | neutral out={NEUTRAL_OUT} down={NEUTRAL_DOWN}"
          f" | CAD-pose knee={KNEE_CAD_POSE_DEG} deg | servo ~{SERVO_DEG_PER_S:.0f} deg/s")
    results = []
    for gait in GAITS:
        for stick in (0.3, 0.6, 1.0):
            r = run(gait, stick)
            results.append(r)
            print(json.dumps(r))
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
    os.makedirs(out, exist_ok=True)
    json.dump(results, open(os.path.join(out, "gait_replay.json"), "w"), indent=1)
    print("\nRead 'knee_deg' against the CAD pose (56.7 deg) and the c=0-is-50-deg-from-stop fact;"
          "\n'touchdown_scoot_cm_s' is the drag the knee absorbs at every footfall.")
