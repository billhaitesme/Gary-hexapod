# The wall deadlock, measured — 2026-08-22

Four isolated candidate mechanisms for "the other legs push" (mids/rears drive
the body up once the fronts bear on rung 2) were built in separate worktrees,
each run through the honest drill (`foot_on_rung_n`, per-rung geom contact AND
foot height in band) with rendered stills, then ranked by a judge agent that
opened the frames against the baseline. Bill's bug report that triggered it,
verbatim: "just holdin one foot on rung 2, twitching, not doing anything else,
not trying to climb at all" / "his other legs aint doin shit".

**Result: nobody won. Nothing climbed.** Every candidate ended worse than the
baseline (nose_end 15.8 / gap_end 13.3 / base_mean 9.7 / base_peak 11.2, no
fault); two faulted belly-down. Patches + stills + reels are in the session
scratchpad (`candA.diff`, `candC.diff`, `ac2.diff`, `candB.diff`).

## What the fan-out measured (these are facts, not theories)

1. **Legs 0/3/4/5 are IK-frozen for 100% of the climb from t≈10 s.** The climb
   stage (door_gym.py ~1150-1205) commands the mids 0.132 m down and the rears
   0.097 m; the IK triangle allows 0.097 / 0.079 (and the sim knee stop only
   0.071 / below-floor). `leg_ik` returns None and the GaitDriver holds the last
   feasible pose. "His other legs ain't doin shit" is literally true. Un-freezing
   them into any feasible, lean-respecting pose is SHORTER than the frozen
   over-reach pose, so the body SINKS (candA runs 1-3: base 11.1 -> 5-8 cm in
   seconds), the pitch drops below the gate and the hands come off.
2. **The ladder rhythm block never ran in the winning run.** Mount pitch peaks at
   0.340 rad = exactly the IMU-phase gate (0.34). foot_lock applied 0.00 of the
   climb, zero front RE-HOOK lines. Every latch/tension/pull-beat/re-hook
   mechanism has been dead code; the fronts reached rung 2 through the stair-gait
   PROMOTE path alone. (ctrlC control run, bit-identical to baseline.)
3. **Hold persistence works physically** (candC run 3): with the holds kept past
   the pitch gate, BOTH fronts honestly bore on rung 2 for 10 s straight
   (foot y 41.5 inside the trough, z 11.7; the y=0.398 pull did NOT drag them
   over the lip) and pulled the body in 3.4 cm (gap 14.8 -> 11.8). Then nothing
   under the body lifted it, the rear sank, a hand slid off, belly-down at 27.5 s.
4. **Hands on rung 2 pin the gap >= ~12.3 cm** (front-leg fold limit
   D >= |L2-L3| = 3.65 cm with the hand near hub height); the mids need
   gap <= ~9.5 to reach rung 1 at all (their re-hooks land on the floor at
   z 0.2-0.3). Geometric deadlock, not a tuning problem. Rung 3 from the current
   body pose is outside the front envelope too (apex stretch 1.13x of L2+L3).
5. **The sim knee cannot open past 101.7 deg interior** (`gary.xml` joint k{i}
   range -45..60 about KNEE0 = 56.7): hub-to-foot maxes at 90.0 mm instead of the
   111.8 mm of a straight leg (rendered check: 56.7 -> 62 mm, 101.7 -> 90,
   140 -> 106, 170 -> 112). That range was never measured. CAD sweep of the
   tibia through the femur bracket (trimesh, pivot hardware excluded): the
   blade-to-bracket gap grows monotonically from ~110 deg to 180 deg
   (0.02 -> 3.0 mm) -- no bracket interference on the opening side; the
   watertight-containment test found zero penetration at every angle but only
   355/2879 femur pieces are closed solids, so it is weak. What CAD cannot see is
   the knee SERVO's horn end-stop (vendor: tibia duty 230 deg + c, +50 deg to the
   2500 us stop in the FOLD direction, ~230 deg of travel the other way).
   **Open question for Bill: can the real Gary straighten his legs?**
6. **The ratchet fitness pays for beat-top touches** ("bears on shelf N" for one
   beat, then collapse): candB's 10-gen ES went 17.5 -> 73.2 on plants while
   nose_end FELL 15.8 -> 13.6 and he ended on the floor. Before any further ES
   the fitness must score the ENDING state (nose_end above the 18 cm tall-stance
   ceiling, mids contact-verified on a rung at the end; final nose below baseline
   = loss). Widened gene bounds were barely used -- the search is not leashed by
   bounds; there is no rise in the mechanism to find.
7. Raising the 0.045 height cap alone (candB isolation run) is a regression:
   drv.height past the envelope saturates IK (held-last) instead of lifting.

## What this means

The body cannot be lifted by the floor legs: the mids sit ~17 mm from their
envelope, the rears at the climbing lean cannot reach the floor. Rise must come
from the HANDS pulling the front end up, and from legs that are ON the ladder.
To get the mids onto rung 1 the body must come within ~9.5 cm, which the fronts
on rung 2 forbid. The way out is posture, not gain: the fronts must hook HIGH
(rung 3-4) with the body pitched steeply (cockroach on a wall, not a ramp):
with the hands well above the hub the front legs fold less, the gap un-pins,
and the mids can take rung 1 while the rears push from the floor. Today's mount
peaks at 19.5 deg of pitch. That is the program: THE STEEP MOUNT -- measure the
pitch ceiling of a mount that plants the fronts on rung 3 directly (Bill's
"shoot UP more, higher arc"), then the inchworm. If the real knee opens to
~170 deg, reach grows 9.0 -> 11.2 cm on every leg and all of the above gets
easier; the sim must be corrected first if so.
