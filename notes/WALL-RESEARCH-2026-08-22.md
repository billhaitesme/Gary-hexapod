# Wall-climbing research notes — 2026-08-22

Commissioned by Bill mid-marathon: "it's like you're not even doing
research." Correct. Here is the research. The literature vindicates
his coaching on every disputed point of the night.

## What actual six-legged climbers do (cockroach studies + robots)

1. **The floor-to-wall transition is a HEAD-FIRST COLLISION, at speed.**
   ~80% of cockroach transitions: run at the wall at ~1 m/s
   (50 body-lengths/s), hit head-first, let the body absorb it
   (~95% of kinetic energy dissipated, restitution 0.22), and be on
   the wall within ~75 ms. No posture preparation beforehand — level
   body until impact. **Bill's ballistic ram was the biologically
   correct entry. Deleting it ("assembly-line nonsense" — the
   nonsense was the sequencing, not the launch) removed the primary
   strategy.**

2. **The slower alternative (~20%) is Bill's other spec, verbatim:**
   run at the wall with the body pitched head-up, use the legs to
   decelerate and climb. Ass down, head up, drive at it.

3. **Interior transitions use the MIDDLE legs to pitch the body up**
   and place the front feet on the new surface. Bill's mids doctrine,
   verbatim biology.

4. **A BODY JOINT materially improves wall transitions** (Stanford,
   ICRA'08 wall-climbing robot). Cockroaches flex; Gary's chassis is
   rigid — this is his real anatomical handicap, and why the wedge
   posture has to fake flexion. (The mesh/CAD confirm: no waist DoF.
   "They can't reach the wall, YET" applies to the spine too.)

5. **It works open-loop.** The DASH hexapod does head-first wall
   transitions at ~80 cm/s with NO sensory input — a 20 mm cone nose
   and momentum. Transition is mechanics, not control.

## What this means for Gary

- Restore the launch as the ENTRY (fast approach -> nose contact ->
  legs catch), feeding the stair-gait as the CLIMB. Collision entry
  and gait climbing are different phases in biology too.
- Momentum budget: Gary is 1.25 kg at ~10-18 cm/s max (sprint data)
  vs DASH 16 g at 80 cm/s — his collision carries proportionally far
  less specific energy. The ram gets him ON the wall, not UP it; the
  climb still needs the support ladder.
- BENCH CAUTION (September): real head-first contact puts the D500
  lidar in the bumper role. Pad the nose or add a bumper standoff
  before any physical transition tests.

## September photo-capture protocol (Bill's idea: "maybe i can make
him do it physically, and you can watch what position his servos are
in")

HPD-0127 servos are write-only — no position readback. Capture by
photograph instead, exactly like the CAD mesh fit:

1. Build the practice rungs (24×24 mm profile, ~45 mm pitch — the
   sweep's number) on a board.
2. Physically pose Gary (powered off) in 5 keyframes: approach,
   nose-contact, front-feet-on-rung-1, mid-climb (fronts rung 2-3,
   mids at wall base), high-climb (fronts rung 3-4, mids on rung 1).
3. Photograph each pose from TWO angles (true side + front-quarter),
   phone level, whole robot + rungs in frame, a ruler or the rung
   pitch visible for scale.
4. I fit the sim skeleton to each photo pair (same point-fit used for
   the CAD meshes) -> real joint-angle keyframes replace guessed
   choreography; the stair gait interpolates between keyframes and
   evolution tunes only timing.

## Restart ritual (standing order)

Every iteration gets RENDERED AND WATCHED before it gets graded.
Reels first, logs second. Add a sprawl detector to the fault laws
(chin propped on the wall must never score as standing).

Sources:
- Transition by head-on collision (J. R. Soc. Interface):
  https://pmc.ncbi.nlm.nih.gov/articles/PMC5832722/
- A body joint improves vertical-to-horizontal transitions (ICRA):
  http://bdml.stanford.edu/twiki/pub/RisePrivate/ReviewsForICRA08/ICRA08_1369_MS.pdf
- Dynamics of rapid vertical climbing in cockroaches (template):
  https://www.researchgate.net/publication/6927872
