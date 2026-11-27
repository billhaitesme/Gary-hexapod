# THE BREACH — full spec (Bill, night of 2026-08-21)

Written down mid-build because two builds died to interrupts/relaunches with
the spec living only in chat scrollback. This file is the authority.

## The door attempt ladder (breach drill)

1. **Three attempts, each a FULL evolution** — "i mean he should reposition
   and then pitch back, and do the whole evolution again." Not a mounted
   re-cock: dismount, back off, reposition, rear up, liberate, slam.
2. **Attempt 3 is the donkey** — "he could do a 180 on his 3rd attempt, and
   perform his forward door-attack, backwards... his LiDAR would help
   counterweight him." Turn 180, close tail-first, mirrored wedge (fronts
   crouch, rears tall), rears pressed on the panel (press mode 4), slam
   backwards. The mast's moment now presses him INTO the door.
3. **The farewell knock** — "before defeat... he gives it one more
   knock(kick)." After the 3rd fail: turn back to face the door, one
   scripted kick_leg strike.
4. **Defeat, performed** — "look it up and down first, then admit defeat by
   lowering and shaking his head back-and-forth in shame. and definitely
   making the defeat noise." Look up/down (body_pitch sweep), sag, sad
   descending tone, then the shame shake.
5. **The shake is his SHAKE, not a turn** — omega at v=0 steps legs one at a
   time ("its just moving its legs around, seemingly one at a time" = bad).
   True wag = body YAW between planted feet: uniform coxa offset, all six
   legs (driver `coxa_bias`). Firmware side: the vendor action set has a
   real scripted shake (CMD_SHAKE_HEAD) — call that on hardware.
6. **Short disengage, then END** — the old walk-away was v=-0.030 until the
   rep clock died (aimless moonwalking, Bill: "why does he back away for
   such a period?"). Now: a body-length back, turn away, rep ends.

## THE YAW-POWERED RAM (Bill: "the YAW! is what will make that battering
## ram action really shine. it should allow for his chassis to be positioned
## fully back, and then thrown forward, with legs planted.")

Mechanism, verified against the driver math: body_dy already sweeps coxas,
but a diagonal leg's RADIAL component of the shift eats the leg_ik envelope
at ~±4 cm — that was the ram's cap. The mids (RADIAL = pure ±x) take a dy
shift as pure tangential coxa arc: hypot(0.071, 0.06) = 0.093, well inside
the 0.112 envelope — ±6 cm each way. So:

- Driver `ram_full` mode: when a leg's IK fails during the throw, binary-
  search the largest per-leg scale that fits — every hip sweeps to ITS OWN
  envelope instead of all hips capped by the weakest geometry. Mids whip
  full-stroke; diagonals contribute their max.
- Cock deeper (dy ±0.060), throw farther (to ∓0.062): ~12 cm of commanded
  runway, "chassis positioned fully back, then thrown forward, feet planted"
  — Bill's "swing the back four hips" spec, literally the hips.

## Standing laws that apply

- Belly on ground 0.4 s = fault, redo. Fall off the wall = restart.
- Metrics lie, frames don't: certify the choreography from rendered frames
  (the ENDING and the MOTION), never from peaks.
- Restart Bill's watcher after ANY edit to loop/drill code.
- Success = door_deg > 16 = a recognizable event: stop slamming, go through.

## Field port (queued)

nav_maze door ladder becomes: knock → (opens? through) → THE BREACH →
(opens? through) → defeat performance → resume search. Firmware EXPLORE has
the voice moments + defeat tone as of 2ce6305; sim defeat shake maps to
CMD_SHAKE_HEAD on hardware.
- Cornering yaw (Bill 2026-08-21: 'it might even help in his cornering abilities... hell yah'): coxa_bias as a body-yaw LEAD while walking — nose points into the corner before the track carves it. Wire into nav_maze during the field port.

## Re-order (Bill, "i know im being pedantic" -- the third act)

New sequence after TWO failed forward evolutions (each = slam + BUMP,
the paws-never-left second shove): look it up and down, PROBE-KNOCK
(watching for a give), quick head shake, then TURN AWAY -- the walk-off
doubles as the mule-kick approach -- close tail-first, square up, and
MULE-KICK it (the reversed mount + slam + bump) as the parting shot.
"it most likely wont [open], but if it does, then he can spin around
quickly, and rush toward the now open door." Fails -> sad descending
tone + head-low walk-off, never looking back. No more turn-back-for-
the-knock dance: tragedy runs 39.7s (was 56.4).

Ajar entry: "if it swings open, but not fully open... he is completely
allowed to give it another knock-probe as he enters" -- press-through
fires up to two knock-probes when the panel reads < 55 deg.

Physics discovered en route: friction tiers CANNOT stop a connected
shove (position-servo displacement, ~16 deg geometric at the panel
mid-span -- resist 3.0/4.5/6.0 all fell identically to the double-tap).
Tier-3 = a REAL lock (hinge range +/-2 deg), run_drill(locked=True).
The single source is class BreachFSM in door_gym.py: the gym drill and
the nav_maze field ladder both drive it.

## The field port + THE VAULT scenario (built 2026-08-21, late)

- maze3=True in build_xml: "THE VAULT" -- new topology, same envelope.
  Spawn room gated by D1 (nvd1, wall y=0.30); the only fast crossing
  east is D2 (nvd2, wall x=1.10) with a crouch-tunnel bypass through the
  same wall (lintel 0.115); exit through D3 (nvd3, east wall y-gap).
  Topology BFS-certified across door states (all-shut sealed / D2-shut
  crouch-only via tunnel / all-open walkable).
- nav_maze: door registry (hinge adr + face-door heading + open sign),
  door_cfg runtime tiers (resist / locked / open), door_banned run-scoped
  ban set, ladder = knock (kick, certified) -> closein (jam the nose) ->
  BreachFSM (the same object the gym drives). Defeat -> BAN + re-search.
  Fresh minds never arm the ladder (no memory) -- run 1 is naive by
  design: push reflex + tunnel. Memory runs arm it via was_free.
- DOOR_SCENARIO (nav_maze, shared with watch_nav --scenario doors):
  RUN 1 first contact: D1 shut t1, D2 shut t2, D3 open. Fresh mind ->
        out/mind_vault.npz.
  RUN 2 the switch: D1 open, D2 shut t1, D3 shut t2. Race on the mind:
        which doors does he try? Kick pops D2; D3 needs the mount.
  RUN 3 the lock: D2 LOCKED (hinge blocked). The full tragedy in the
        field -- look, probe-knock, shake, mule-kick, sad walk-off --
        then BAN, re-search, and the tunnel is the second act. D3 open.
- Bill's windows: watch_nav.py --scenario doors

## The ajar-door shove re-station (BUILT -- was queued, promoted after
## RUN 2 saw a 0.02-friction door defeat six slams that all hit air)

The square-up shove stations on the GAP centre; an ajar panel has
rotated toward its hinge, so field shoves half-miss above ~35 deg at
high friction (tier 0.6 field-defeated despite gym cert). Fix: at
stuck-shove, re-station toward the panel's CURRENT midpoint (nav knows
hinge pos + angle). Scenario tiers meanwhile: 0.02 / 0.25 / locked.

Resolution: BreachFSM phase 92 -- after squaring, strafe (drv.vx) onto
the panel CURRENT midpoint before mounting; the caller supplies
shove_lat() (nav computes hinge + rotated half-length; the gym passes
None and keeps certified behavior bit-for-bit). Plus: stale suspects at
doors standing open (>40 deg) slate-and-replan instead of mounting or
legacy-popping them.

## The remaining frontier, now with a reproduction (end of the marathon)

RUN 2 (the switch), current build: opening act is CLEAN -- four
sweep-reopens of the brushed entry door (~5 s each, dispatch bug fixed:
the legacy branch had been clobbering the resweep stage same-tick),
door curiosity finds nvd2 at 88 s. Then at ~103 s, after a push on
D1's panel, he ends up PHYSICALLY WEDGED at (+0.10, +0.13) -- zero
displacement, marching in place, DEADMAN recovery (backing) cannot
free a body hung on a panel edge. Bill saw this live ("marching in
place for a bit").

This is the named frontier -- half-open panels as dynamic obstacles --
in its physical form: entanglement. Next session's opener: frames of
the wedge (what exactly is he hung on), then an UNTANGLE reflex
(the gym's getup/redo machinery is the seed: detect wedge = commanded
walk + zero displacement + non-level attitude, respond with the
getup-style splay + lateral extract, not a straight back-off).
