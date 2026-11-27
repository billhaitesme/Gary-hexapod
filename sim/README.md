# Gary sim — get the gaits right virtually before touching servos

Bill's ask (2026-08-18): "is there an onnx trainer or something we could use to get it right
virtually first? so it saves on his servos? and plug-flash-unplug time."

Short answer: ONNX is a model *file format*, not a trainer — the thing that saves servos is a
**physics simulator**. We are unusually well placed for one because `cad_tools/` extracted exact
link lengths, joint axes and mount positions from Hiwonder's own CAD (`gary_geometry.h`).

## Stages

### Stage 1 — kinematic replay (DONE, `gait_replay.py`)
Pure-math port of `run_custom_gait_scheduler` + CAD geometry. No hardware, runs in 2 s.
Measures commanded joint envelopes, joint rates vs servo capability, touchdown scoot, IK failures.
First results (`out/gait_replay.json`, 2026-08-18):
- **The swing lift is the abuse.** Peak commanded femur rate ~810–850 °/s and knee ~400–450 °/s in
  every gait — beyond a ~500 °/s micro servo — because `CUSTOM_GAIT_LIFT_CM 2.5` is squeezed into a
  200 ms swing window (tripod & wave both land at 200 ms). The servo physically can't follow, lags,
  and takes the error as load. Forward speed barely moves these numbers — halving speed caps (the
  July band-aid) never addressed it.
- **Touchdown scoot = full body speed** in tripod (3.5 cm/s at stick 1.0), small in wave/ripple only
  because their speed caps are tiny. Confirms the firmware comment's diagnosis.
- Knee interior angle stays 31–64° — comfortably away from the L2+L3 straight-leg singularity, and
  brackets the CAD pose (56.7°). The *danger* end (fold past the stop) isn't reached by clean gait
  math; it's reached by IK fed with garbage targets — which is what the knee clamp now catches.

Candidate fixes to test in stage 2 (in order): longer swing window / slower cadence rather than
smaller speed caps; lift lowered ~2.5 → 1.5 cm; velocity-matched touchdown (blend the last ~20% of
swing X into the stance slope so the foot lands moving with the ground).

Assumption to verify on bench: vendor amplitude convention `amp = v*T/2` (affects stride length,
not the rate/scoot story).

### Stage 2 — MuJoCo digital twin (DONE 2026-08-18, `gary_mujoco.py`)
Build a MJCF model from the CAD: bodies + joint frames from `cad_tools/out/leg_geometry.json`,
meshes exported per part from the .3dm via `rhino3dm` (or plain capsules first — fine for gait
work), servo actuators modeled as position servos with velocity/torque limits (bench: measure
stall torque or use Hiwonder's spec). Then replay the same scheduler inside physics and read
**knee torque, stall duty-cycle, slip, body wobble** directly. Tune cadence/lift/beta/touchdown
against those. `uv run --with mujoco` — no install drama, has a viewer for eyeballing gaits.

### Stage 3 — optimization / learning (OPTIONAL, after stage 2 matches reality)
For a hexapod, CMA-ES over the ~8 gait parameters inside MuJoCo beats deep RL for effort-to-result.
If we ever *do* train a policy (e.g., terrain-adaptive gait), that's where ONNX finally appears:
export the trained policy to ONNX and run it on the Pi 5 co-processor (not the ESP32 — no room).
The sim-to-real gap is real; anything stage 3 produces still gets a supervised bench intro with the
knee guard armed.

## The loop this enables
sim finds parameters → parameters go into `gary_config.h` / scheduler constants → ONE flash per
bench session instead of flash-test-guess-reflash — with the knee guard as the hardware backstop.


## Stage 2/3 results (2026-08-18, first day)

`gary_mujoco.py` builds gary.xml from the CAD numbers (18 position-servo joints, capsule links,
estimated masses/torque — bench items). `--view` opens a live interactive window (real-time paced);
`--video x.mp4` renders offscreen. `train_gait.py` runs CMA-ES over {cadence, lift, speed_cap}
scored as speed minus knee-torque-saturation (the burnout proxy), on the velocity-matched swing.

Findings so far:
1. With firmware-equivalent parameters the twin initially reproduced "marches in place / drifts
   backward" — but the cause in sim was a FK sign inversion in the driver (swing drove feet INTO
   the floor). Kept in the log as a lesson: the same symptom had two different causes (sim bug vs
   real touchdown-scoot); always FK-verify before trusting a twin.
2. After the fix: tripod walks +5.3 cm/s, wave/ripple hit their commanded speeds.
3. **Velocity-matched touchdown** (cubic Hermite swing whose endpoint velocity equals the stance
   sweep — replaces the cosine) cuts tripod knee saturation ~17% -> ~10%. This is the trajectory
   change to port into `run_custom_gait_scheduler` after bench validation.
4. Standing at stick 0 with the scheduler still cycling = marching in place with full lift — the
   lift arc alone saturates knees. Firmware note: don't run the scheduler when velocity is ~0.
Training outputs land in `sim/out/train_<gait>.json` (best {cadence, lift, speed_cap} per gait,
mapped 1:1 to GAIT_MIN_CADENCE / CUSTOM_GAIT_LIFT_CM / GAIT_SPEED_CAP).
