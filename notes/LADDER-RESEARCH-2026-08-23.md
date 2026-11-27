# How six-limbed robots actually climb — research, 2026-08-23

Bill, 04:50: "maybe go do some more research, maybe someone has gotten a
hexapod to climb a ladder?" Someone has. This note is what they did, and
what it says about what I have been doing wrong all night.

## 1. ASTERISK (Osaka University, Arai lab)

A six-limb "limb mechanism robot": 4 kg, 840 mm span, body 78 mm tall,
standard body height 180 mm, legs extended 210 mm; 4 joints per limb,
Dynamixel DX-117 position servos with **settable joint compliance**.
It climbs a 230 mm step (higher than its body) and climbs ladders.

### 1.1 Step climbing (Fujii, Inoue, Takubo, Arai — ISARC 2006)

Wave gait, "at least four legs are always supporting the body", legs
moving hind → front. Their sequence, verbatim:

1. "The robot puts the mid legs forward and supports its body with the
   mid and hind legs."
2. "The robot raises the front legs aside, and inclines its body to match
   the height of the step. Then the robot raises the front legs without
   hitting the step."
3. "The robot puts the front legs on the step."
4. **"The robot moves its body forward and upward until the center of
   gravity comes between the front legs and mid legs."**
5. "The robot puts the hind legs forward as much as possible."
6. "The robot puts the mid legs on the step."
7. "The robot moves the body forward and upward until the center of
   gravity goes beyond the mid legs."
8. "If the front or hind legs are stretched to the limit in the process
   of 7, the robot puts the limited legs forward. Furthermore, where the
   robot cannot maintain its stability if it raises the right leg and
   left legs at the same time, the robot puts the legs one by one."
9. "If the center of gravity comes beyond the mid legs, the robot lifts
   its body above the height of the step and makes its body horizontal."
10. "The robot puts the hind legs on the step."
11. "The robot puts the mid legs forward and the step climbing is
    completed."

Two instruments make it work on hardware:

- **Contact by compliance** (their Fig. 6): soften the joints, lower the
  leg, read the actual angles, forward-kinematics the foot, compare to
  the commanded foot; an error > 7 mm = contact; then set the reference
  to the current angles and harden. No foot sensors.
- **Stability margin** before lifting any leg: Hirose's Normalized Energy
  Stability Margin, with the body inclination from the accelerometer
  (the IMU). Small margin → move legs one at a time.

Their workspace analysis: inclining the body opens the reachable region
for the CoG; it collapses past ~45° because of joint limits. They choose
the inclination that maximises that region for the given step height.
240 mm failed only because the legs were at their stretch limit and the
body could move so little per cycle that it took > 2 minutes.

### 1.2 Ladder (Advanced Robotics 24(11), 2010 — abstract only, paywalled)

"The upper three legs hold on to the upper rung from both sides
alternately, just like pinching, with legs that cannot support forces
controlled to always touch the rungs slightly in order to pinch the rungs
with the other legs." — i.e. a limb that cannot bear still stays in light
contact so the bearing limbs can pinch against it.

## 2. Humanoid ladder climbing (Luo et al., ICRA 2014, DRC-Hubo)

Same skeleton, stated as **motion primitives, one hold change at a time**:
"Such primitives invoke one hold change: add or remove the contacts of
one limb while keeping other holds unchanged", interleaved with
**CoM-transfer primitives** that "transfer the COM position without
changing the stance" — "before moving left foot onto a higher rung, the
COM is transferred to the right foot." One climbing cycle that ascends a
rung: mountLH, mountRH, moveCOMtoRF, placeLF, moveCOMtoLF, placeRF,
moveCOMtoCenter, placeLH, placeRH. Compliance on the contacting limbs
absorbs ±2 cm of position error.

## 3. What this says about Gary

Bill's doctrine, in order, IS this method: ass down head up (incline the
body to the step), plant feet and use legs, one leg at a time, everything
works together like a gait, mids extend under the body to support and
push, hold the wall only once you are on it.

What my controller has been missing, named by both papers:

1. **The body-motion phase.** After a limb is placed, the BODY moves —
   forward and upward, on all the planted feet, as a commanded body
   trajectory — until the CoG sits between the new hold and the next
   support. I place hands and then try to *pull* on them (tension
   targets on one leg). They move the body on its legs. That is the
   difference between a climb and the "twitching" Bill sees.
2. **Alternation: move body / move one limb.** Every limb transfer is
   preceded by a CoM transfer that unloads it. My strikes fire on a
   timer with the weight wherever it happened to be.
3. **Re-place limbs that hit their limit** as the body advances (their
   step 8) — the rears "put forward as much as possible" are the
   rubber-feet walk-in I never built; the mids go up when the body has
   passed them, not on a rung-count.
4. **Inclination as a workspace choice**, not a pitch target: choose the
   body angle that maximises the reachable region for the next hold.
5. **Contact by compliance** on the real Gary: the Hiwonder bus servos
   read back position; soften → lower → compare → harden is the
   September port of `foot_on_rung_n`.
6. **A limb that cannot bear still touches** (light contact) so the
   others can pinch against it.

## 4. Implementation plan (the primitive machine)

Replace the rhythm/pull block with a small state machine of primitives,
each ending on a measured condition (honest contact, CoG position from
the model, reach limits from IK):

- `INCLINE(theta)` — body pitch chosen for the next hold's workspace
- `PLACE(limb, rung)` — the existing hook strike (diagonal, press,
  honest plant, return-on-miss)
- `BODY(dy, dz)` — move the body forward/up on all planted feet (the
  GaitDriver's body_dy/height ramps with the planted feet as world
  anchors) until the CoG crosses the target line or a leg hits its IK
  limit
- `ADVANCE(limb)` — put a limit-hit floor leg forward (lifted step,
  rubber feet)
- `STABLE?` — NESM-style margin from the support polygon + IMU pitch
  before any lift; if small, one limb at a time

Cycle for one rung: PLACE(front L, n) → PLACE(front R, n) → BODY(+, +)
→ ADVANCE(rears) → PLACE(mids, n-1) → BODY(+, +) → [legs at limit →
ADVANCE] → PLACE(fronts, n+1) → …

## Sources

- Fujii, Inoue, Takubo, Arai, "Climbing up onto Steps for Limb Mechanism
  Robot ASTERISK", ISARC 2006 —
  https://www.iaarc.org/publications/fulltext/isarc2006-00096_200605312158.pdf
- "Ladder Climbing Method for the Limb Mechanism Robot ASTERISK",
  Advanced Robotics 24(11), 2010 —
  https://www.tandfonline.com/doi/abs/10.1163/016918610X512596
- "Ladder climbing control for limb mechanism robot ASTERISK" (ICRA 2008) —
  https://www.researchgate.net/publication/224318688_Ladder_climbing_control_for_limb_mechanism_robot_ASTERISK
- Luo et al., "Robust Ladder-Climbing with a Humanoid Robot with
  Application to the DARPA Robotics Challenge", ICRA 2014 —
  https://motion.cs.illinois.edu/papers/ICRA2014-Luo-ladderclimbing.pdf
- Vaillant et al., "Multi-contact vertical ladder climbing with an HRP-2
  humanoid", Autonomous Robots 2016 — https://hal.science/hal-01276931
- "Energy-efficient narrow wall climbing of six-legged robot" (ASTERISK),
  ROBOMECH Journal 2018 — https://doi.org/10.1186/s40648-018-0121-y
