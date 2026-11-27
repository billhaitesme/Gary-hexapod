# CAD ground-truth geometry vs the firmware — 2026-08-18

Source: `3. Technical Parameter & Diagram\1. Diagram\minihexa3 003.3dm` (Hiwonder's full build model,
Rhino 8, mm, 947 parts). Scripts: `cad_tools/dump_parts.py` → `leg_geometry.py` → `foot_tip.py`;
raw results in `cad_tools/out/*.json`; constants in `firmware/GARY_v3/gary_geometry.h`.

Method: servo shaft axes = line through the white output hub and the 20 mm horn disc (disc normal
aligns with the servo body's 14 mm axis at cos 0.99); coxa servos = vertical shafts; each remaining
servo assigned to its nearest coxa; link lengths = axis-to-axis distances (independent of how the legs
are posed); foot = orange rubber pad's bottom face centre; all six legs agree to 0.01 mm.

## 1. Result: the vendor's numbers were already right

| Quantity | CAD | Vendor `kinematics.h` / `hiwonder_robot.h` | Δ |
|---|---|---|---|
| Coxa L1 (coxa axis → femur axis) | **29.20 mm** | 28.5 (`BODY_TOP_LEN 2.85`) | +0.7 |
| Femur L2 (femur axis → knee axis) | **37.83 mm** | 37.0 (`TOP_MIDDLE_LEN 3.70`) | +0.8 |
| Tibia L3 (knee axis → foot pad bottom centre) | **74.35 mm** | 72.0 (`MIDDLE_BASE_LEN 7.2`) | +2.4 |
| Middle-leg coxa radius | 43.55 | 43.4 (`LEG2_START_X 4.34`) | +0.15 |
| Corner-leg coxa position | (31.32, 57.00) → r 65.0, az 61.2° | (31.5, 56.5) → r 64.7 | <0.5 |
| Femur ∥ knee axes / coxa ⟂ both | 0.0° / 90.0° | assumed | — |
| `JOINT_POS_OFFSET 1.5` | no CAD counterpart found (foot is in the leg mid-plane; blade is straight along axis→foot within 3 mm) | 15 mm | ? |

The April `MINIHEXA_REBUILD` sketches used the same 2.85/3.70/1.50/7.20 numbers, and the live
GARY_v3 (including the custom wave/ripple scheduler) still calls the vendor `RobotLeg::move()` →
`ikine()` → `set_angle()`. **So there was never a link-length mismatch of more than ~2 mm anywhere in
Gary's history.** The "wrong link lengths fried the knees" hypothesis is retired — but not the IK rewrite
itself. Bill (2026-08-18): during the April rewrite "all the tucking lock-ups and servo grinding" very likely
did the first damage. That is consistent: `MINIHEXA_DIAG` records the lib's "C-joint output is always out of
range" and the first Gary flash stood with tucked legs — i.e. knees commanded into/past their end stops and
held there (see §2.1). Best reading: **damage accumulated in April (range/convention mismatch → stalls),
finished in July under the wave/ripple scheduler.**

## 2. What the CAD + vendor code *do* point at for the knees

1. **The knee servo runs 50° from its hard stop with no clamp.** `hiwonder_robot.cpp:153-157`:
   coxa = 140°+a, femur = 140°−b then hard-clamped to 857–2143 µs, **tibia = 230°+c with no clamp** on
   a 280° servo → zero point 2143 µs (mirrored legs 857), only 50° of headroom before 2500 µs. Any IK
   answer with c > +50° (foot pulled in/under, or lifted high) is silently pinned at the end stop and
   *held there under load* — the classic Hiwonder stall-burnout case. The femur got a clamp; the knee
   didn't. First fix: clamp knee duty (start with 650–2350 µs, verify on the bench) and log any clip.
2. **The custom scheduler drives `legs[i]->move(target, 0)`** — zero-time (no interpolation) per-leg
   targets from a 1 ms loop (`LOOP_DELAY_MS 1` since Jul 13, was 50). The vendor path smooths with a
   time argument; ours slams. Combined with the documented "touchdown-scoot" (foot lands with body
   velocity, servo can't stop it, foot drags) that is sustained stall-current on the knee at every
   touchdown, 5–6 times per wave/ripple cycle. Bill's own read ("wave/ripple still unrefined") fits.
3. Standing pose in the CAD has the knee folded to 56.7° interior (femur 45° up, tibia 11.7° from
   vertical, body 44.5 mm off the ground). That's a high-torque knee posture: the tibia is nearly
   vertical so the knee holds ~all of the leg's share of body weight through a short lever. Sitting
   in this pose for long periods on weak/hot servos is itself a burnout path (`balance(true)` idle).

Servo hardware note: 13 × 14 × 32.5 mm body, shaft on the 14 mm axis, 20 mm round horn — Hiwonder's
own PWM servo, spares = the 16 in the Standard donor bin. Not MG90S.

## 3. What this changes in the plan

- Drop "rebuild IK from CAD" as the first job. Keep the vendor IK (it's correct); put a **protection
  layer** around it: knee duty clamp + clip counter, femur clamp already exists, a **stall watchdog**
  (if a leg's commanded target is unreachable / clipped for > N ms, back it off), and per-gait
  `move()` times > 0 in the custom scheduler instead of 0.
- Then refine wave/ripple: replace the halved-speed band-aid with a touchdown that arrives at zero
  world velocity (retract the swing target by the body displacement over the last few ms).
- Bench measurements still needed (hardware in Norway): (a) run `MINIHEXA_DIAG`-style FK/IK sweep to
  learn what physical knee angle c = 0 (2143 µs) is and where the blade actually contacts the femur
  bracket at each end; (b) record the all-1500 pose and the CAD reference pose duties; (c) confirm
  where `JOINT_POS_OFFSET` shows up (probe `fkine()` on device with the DIAG sketch — the lib is
  precompiled, that's the only way).

## 4. Files

- `firmware/GARY_v3/gary_geometry.h` — the constants (not yet `#include`d anywhere).
- `cad_tools/out/parts.json` — every object: layer, centroid, AABB, PCA extents/axes.
- `cad_tools/out/leg_geometry.json`, `foot_tip.json` — per-leg axes, lengths, posed angles.
