"""THE NAVIGATOR (Bill, 2026-08-20: "that's what we should be doing!").

Gary builds an OCCUPANCY GRID in his head from the lidar ring as he walks —
free / wall / unknown, 0.15 m cells — and NAVIGATES: route to the best frontier
(free cell touching unknown), walk the route, repeat. When the map says "fully
explored and no way out," he walks up to the nearest wall he can't explain and
fires the GIMBAL SCAN there; a tunnel verdict is carved into the map as LOW
cells the planner may route through (crouching). Beetle -> navigator.

This is July's "Gary Live Map" pipeline moved INSIDE his head: same ring, same
pose, but now HE reads the map. Grid cost at real scale: a 6x6 m room at 15 cm
is ~1.6 KB — ESP32-trivial. (Sim uses true pose; hardware uses odometry, and
the map itself becomes the drift fix via scan-matching later.)

v2 (the motion-loop fix). What was broken and how it's fixed:
* Self-echo poisoning: knee swing-tops cross the ring plane; mapping now runs
  through the per-bearing rolling MIN/MAX window (the scout-mode filter, ported)
  — carve free to the window MIN, wall evidence at the window MAX, window
  cleared while pivoting (bearings only line up when he's tracking straight).
* Walls flickered FREE: free-carving could stomp a WALL cell. Now wall belief is
  signed EVIDENCE (+4 normal hit, +1 in the 0.11-0.16 ambiguous knee shell and
  only while planted, -2 when a ray passes clean through). A cell is WALL at
  evidence >= 8. See-through decay = belief revision — fresh sensing overrides
  memory, which is exactly the muscle the shifting labyrinth will train.
* Frontier whipsaw: nearest-frontier ties flipped the goal every replan. The
  planner is now one Dijkstra (step cost prefers corridor centers over
  wall-hugging cells) and frontiers are SCORED: distance + a turn penalty on
  the first step direction - a keep-the-current-goal bonus - an unknown-area
  bonus. Goals that get reached but never resolve (or hold too long) are BANNED.
* Dead stop check: the old front-arc measure filtered out every ray that could
  ever trip it. Now raw rays + rolling max, stop at 0.17 m, and a stuck
  detector (no displacement for 8 s) that bans the goal, bumps wall evidence
  ahead, and backs off.
* Blind scanning: the lintel fingerprint physically needs the dipped ray to
  reach the wall before the floor (~0.6 m) — map exhaustion now routes him TO
  the nearest unexplained wall (wall cell with unknown beyond) and scans there;
  scanned standpoints are remembered so the next scan tries a different wall.

Run:  uv run --with "mujoco<3.12" --with numpy python nav_maze.py    (headless)
      (pin: mujoco 3.12.0 shipped mid-session with no Windows wheel — the
       whole record book ran on 3.11.0)
      ... --dbg / --view / --video out/x.mp4
"""
import argparse, heapq, json, math, os, time
from collections import deque
import numpy as np
import mujoco
import gary_mujoco as G
import door_gym as DG          # BreachFSM: the gym-certified door choreography

HERE = os.path.dirname(os.path.abspath(__file__))
FINISH_X = 2.42        # planning target (route cells beyond the wall)
FINISH_MAST_X = 2.32   # the FINISH: lidar unit past the wall's east face
SIM_MAX_S = 900.0
CTRL_EVERY = 5
CAD = 493

CRUISE = dict(speed=0.85, lift=0.022, height=0.020)
DUCK   = dict(speed=0.60, lift=0.022, height=-0.020)
EXP_PIPE_LIFT = float(os.environ.get("PIPE_LIFT", "0.016"))  # in-pipe swing cap

# TIME-TRIAL genes (curriculum stage 1): the knobs evolution is allowed to
# push. Defaults = the 6-generation time-trial winner, 286.2 s vs the 407.6 s
# hand-tuned first solve (out/nav_trials.json) — the big find was a SLOWER
# crouch (0.60 -> 0.45): rushing the tunnel caused the mid-lane grind-stall.
# Re-evolved 2026-08-21 AFTER the wide-cornering fixes: with snags gone,
# evolution floored the cruise gene (0.85 -> 1.0) — speed stopped being
# punished at corners. 332.9 s baseline -> 161.8 s record (beetle: 824).
# Final overdrive evolution winner (10 gens, knee-priced fitness):
# 154 s base maze at 38% knee sat — faster AND kinder than the old 161.8 s
# record (51% sat). Stride does the speed; posture pays the knees.
# Final re-tune on the finished mechanics (phase-continuous gearbox, zoomies
# gate, climb integrity): 141 s base maze — evolution put maze CRUISE in the
# sprint band (2.11x) and trusts slow-in-fast-out to guard the corners.
TUNE = dict(cruise=1.495,  # stride fraction — cruising in the sprint band
            duck=0.865,    # crouch-lane speed (quicker = less time folded)
            replan=1.539,  # replan cadence (s)
            carrot=5,      # lookahead cells offered to the LOS check
            kyaw=1.735,      # heading PD gain
            fwd_full=0.6,  # |yaw err| below which he walks full speed
            fwd_stop=0.921,# |yaw err| above which he pivots in place
            cadence=420,   # tripod cycle ms
            lift=0.015,    # cruise foot lift (m) — flat-floor optimum is ~1cm;
                           # lift only pays for terrain clearance
            brake_at=0.20,   # slow-in-fast-out: steering rate where the
            brake_slope=2.2,
            crab=0.0,      # APEX momentum-transfer gain (0 = legacy cornering) # stride brake engages, and how hard. RULE-TURNED-
                             # GENE (Bill: "was it a rule, or something it
                             # decided? it was good before it started to
                             # drift") — slope 0 = brake OFF, drift carries it
            zoom=2.0,      # ZOOMIES stride (open-straight sprint; drag-strip
                           # envelope: 2.0x@420ms = 10.5 cm/s at 12% knee sat,
                           # the sustainable ceiling — 3.5x hits 18.4 but pins
                           # the clutch)
            tilt_slow=0.7, # speed multiplier while tilted >0.10 rad — care under
                           # a moving floor. Graduation-Gary evolved this reflex
                           # (0.77-0.95); defaulting it OFF let the tipping
                           # plate flip him at full speed. Race evolution may
                           # re-price it.
            low_extra=0.0) # extra route cost per LOW cell (shortcut appetite)

# PANIC profile (Bill: "how fast if it NEEDS to") — pure-clock evolution, no
# knee pricing: 87.8 s full unknown maze (9.4x the beetle). The knee clutch
# screams the whole way; this is the something-is-chasing-me button, NOT a
# default. Run with --panic.
PANIC = dict(cruise=2.435, duck=1.338, replan=1.0, carrot=4, kyaw=2.235,
             fwd_full=0.593, fwd_stop=1.012, cadence=467, lift=0.026,
             zoom=2.085, brake_at=0.136, brake_slope=1.198, crab=0.014)
# (v12 CLEAN CLASS: 83.2 s, ZERO wall contact — first contact-free panic
# run. Glances priced at 2 s each; evolution chose none: the old 74.5
# bought 0.21 s per wall strike, 41 strikes. Cruise DOWN 3.12 -> 2.44,
# replan at the 1.0 bound, apex-crab declined (0.014 ~ off): a stepper
# corners clean with speed discipline, not drift. 74.5 stands as the
# unlimited-class number.)
# (v11: 74.5 s — full 10-gen run at Bill's insistence; found at gen 6 then
# FOUR dry generations = converged for these mechanics. Cruise 3.12 is
# still shy of the 3.6 bound: control, not the cap, is the limit.)
# (v10: 75.1 s — on one-squat + final-approach-curve mechanics. Each
# behavior cleanup freed time only a re-adapt could spend: 85.9 genes on
# these mechanics -> cleanup -> re-adapt has now run three cycles.)
# (v9: 85.5 s — single-aimed-peek + no-pirouette mechanics (Bill's reel
# autopsy). Evolved vs the honestly re-scored incumbent after the
# resume-ghost fix: the v8 genes actually run 86.1 there.)
# (v8: 85.7 s — the first record on the mouth-mastery mechanics. The old
# 85.9 genes score 91.1 there (doorway manners cost raw recklessness ~5 s);
# evolution won it back with MORE cruise (2.845) and a GENTLER brake —
# discipline at the mouths buys abandon in the open.)
# (v6: 85.9 s. The crawl verdict: duck 1.338 — evolution took the hot-crawl
# offer PARTWAY through the clutch slip, then stopped where control holds.)
# (v4, on the final mechanics incl. tunnel-straight: 121.0 s. Note the moral:
# it kept the STANDARD stride and just sharpened cadence/turn thresholds —
# once cornering got disciplined, careful and reckless nearly converged.
# History: 87.8 s pre-swing bulldozer, 95.2 s nose-pivot, 121.0 s final.)

CS = 0.15                      # cell size (m)
# Grid origin PHASE matters more than size: at -0.5 every wall centerline in
# this maze (all at 0.05 mod 0.15) fell 5 mm off a cell center — worst case,
# every wall column a flickering wall/corridor MIXED cell. At -0.475 the wall
# centerlines sit ON cell centers: wall cells stay walls, corridors stay clean.
GX0, GY0, GNX, GNY = -0.475, -0.475, 24, 18   # grid origin + dims (covers the maze)
UNK, FREE, WALL, LOW = 0, 1, 2, 3
RAY_MAX = 1.2                  # trust ring geometry out to this range
SELF_MIN = 0.16                # returns closer than this may be his own knees
SELF_HARD = 0.11               # returns closer than this are ALWAYS his own legs
WALL_EVID = 8                  # evidence units for WALL
FRONTIER_EVID = 4              # cells with this much wall evidence can't be frontiers
RING_WIN = 6                   # ~0.3 s rolling window (self-echo blips vanish)
REPLAN_S = 2.0
GOAL_MAX_HOLD_S = 25.0
BAN_S = 90.0                   # long enough to try elsewhere, short enough to forgive
MAST_FWD = 0.0555              # lidar mast forward of body center
SURVEY_BOUNDS = (-0.35, 2.36, -0.25, 1.82)   # survey interrogates the ARENA:
# chasing "walls" at the grid edge outside the maze sent him on an endless
# peek/carve/revoke frenzy in the void


def yaw_of(q):
    return math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def cell_of(x, y):
    return (int((x - GX0) // CS), int((y - GY0) // CS))


def world_of(c):
    return (GX0 + (c[0]+0.5)*CS, GY0 + (c[1]+0.5)*CS)


def in_grid(c):
    return 0 <= c[0] < GNX and 0 <= c[1] < GNY


N4 = ((1, 0), (-1, 0), (0, 1), (0, -1))


class Mind:
    """The onboard map + planner."""
    def __init__(self):
        self.grid = np.zeros((GNX, GNY), dtype=np.uint8)   # UNK
        self.evid = np.zeros((GNX, GNY), dtype=np.int16)   # signed wall evidence
        self.path = []
        self.goal = None
        self.banned = {}       # frontier cell -> banned-until t
        self.carves = []       # carved LOW lanes (lists of cells), revocable
        self.carve_geo = []    # per-carve sub-cell centerline (px, py, ax, ay):
        # the peek/gimbal measured the opening's true center — the carve used
        # to keep only the 15 cm cell trail and lane_center re-derived a line
        # from wall CELL CENTERS (0.725 for the 0.53-0.87 tunnel: Bill's 2-4 cm
        # north bias, "left foot rides the wall line"). Keep the measured line.
        self.carve_edge = []   # per-carve edge bracket, refined by CROUCHED
        # rays only (the ring is below the lintel, so endpoints are the true
        # flanking faces — tall/raised rays hit the lintel INSIDE the opening
        # span and would poison the bracket): [hit_n_min, hit_s_max, pass_max,
        # pass_min, n_hit_n, n_hit_s, n_pass]. Flank hits bound the opening
        # from outside, pass-throughs from inside; the midpoint of the two
        # brackets beats the peek's 15-deg bearing quantization (which put
        # carve lines 4-5 cm off the true center in either direction).
        self.lowdoubt = np.zeros((GNX, GNY), dtype=np.int16)  # standing rays
        # seeing clean through a LOW cell = evidence the crouch verdict is
        # stale (a tall doorway or open floor mis-carved). True lanes are
        # immune: standing rays END on their lintel, and confirmed carves
        # carry endpoint evidence (the evid gate in _see_through).
        self.demoted = 0       # LOW cells purged by doubt/sanitize this run
        self.low_extra = 0.0   # route-cost surcharge per LOW cell (risk appetite)
        self.bump = np.zeros((GNX, GNY), dtype=np.int16)   # CONTACT no-go layer:
        # things he physically hit that the lidar can't see (below the ring
        # plane, e.g. the tipping plate). Ray see-through must NOT erase it —
        # it decays slowly with TIME instead, so he may eventually retry.

    def save(self, path):
        """The map IS memory: persist what he learned for the next run."""
        np.savez(path, grid=self.grid, evid=self.evid, bump=self.bump,
                 carve_geo=np.array(self.carve_geo, dtype=np.float32).reshape(-1, 4),
                 carve_edge=np.array(self.carve_edge, dtype=np.float32).reshape(-1, 7))

    @classmethod
    def load(cls, path):
        m = cls()
        z = np.load(path)
        m.grid = z["grid"].astype(np.uint8)
        m.evid = z["evid"].astype(np.int16)
        if "bump" in z:
            m.bump = z["bump"].astype(np.int16)
        if "carve_geo" in z:
            m.carve_geo = [tuple(row) for row in z["carve_geo"]]
        if "carve_edge" in z:
            m.carve_edge = [list(row) for row in z["carve_edge"]]
        while len(m.carve_edge) < len(m.carve_geo):
            m.carve_edge.append([1e9, -1e9, -1e9, 1e9, 0, 0, 0])
        m.sanitize()
        return m

    def sanitize(self):
        """Purge stale false-LOW breadcrumbs on load (Bill's clean-survey ask):
        flood-fill each LOW cluster; a cluster with any cell OUTSIDE the arena
        was a carve through the outer wall into the void (the stale survey had
        a 20-cell phantom crouch-highway along the south wall at y=-0.25) —
        demote the whole cluster to UNKNOWN so real evidence reasserts itself.
        The finish side (east of the exit wall) is legitimately outside."""
        seen = np.zeros((GNX, GNY), dtype=bool)
        for i in range(GNX):
            for j in range(GNY):
                if self.grid[i, j] != LOW or seen[i, j]:
                    continue
                stack, cluster, void = [(i, j)], [], False
                seen[i, j] = True
                while stack:
                    c = stack.pop()
                    cluster.append(c)
                    wx, wy = world_of(c)
                    if wy < -0.14 or wy > 1.74 or wx < -0.24:
                        void = True
                    for dx, dy in N4:
                        nc = (c[0]+dx, c[1]+dy)
                        if in_grid(nc) and self.grid[nc] == LOW and not seen[nc]:
                            seen[nc] = True
                            stack.append(nc)
                if void:
                    for c in cluster:
                        self.grid[c] = UNK
                    self.demoted += len(cluster)

    def doubt(self):
        """The world CONTRADICTS the map (boxed with no exit in a fully-known
        maze): halve all wall evidence; walls that drop below threshold demote
        to UNKNOWN — not free, unknown — so the frontier machinery goes back
        and LOOKS. Real walls re-confirm in a second; a moved wall doesn't.
        (Bill's shifting-labyrinth spec: fresh sensing overrides memory.)"""
        self.evid //= 2
        demote = (self.grid == WALL) & (self.evid < WALL_EVID)
        self.grid[demote] = UNK
        self.banned.clear()
        return int(demote.sum())

    # ---------------- sensing ----------------
    def _wall_evidence(self, c, amt):
        self.evid[c] = min(self.evid[c] + amt, 30)
        if self.evid[c] >= WALL_EVID and self.grid[c] != LOW:
            self.grid[c] = WALL

    def _see_through(self, c, tall=True):
        # a ray passing clean through erodes wall belief: fresh sensing
        # overrides memory (the shifting-labyrinth muscle). Endpoint evidence
        # (+4) dominates grazing decay (-1), and WALL only demotes near zero —
        # hysteresis, so wall/corridor MIXED cells don't flicker.
        if self.grid[c] == LOW:
            # LOW-DOUBT: a STANDING ray passing clean through a crouch lane
            # says "this is walkable tall" — a stale/false verdict (doorway,
            # open floor). A TRUE lane never fires this: standing rays end on
            # its lintel, so its evid stays high (confirm + traverse hits).
            if tall and self.evid[c] <= 2:
                self.lowdoubt[c] += 1
                if self.lowdoubt[c] >= 40:
                    self.grid[c] = FREE
                    self.lowdoubt[c] = 0
                    self.demoted += 1
            return
        if self.evid[c] > 0:
            self.evid[c] = max(self.evid[c] - 1, 0)
        if self.grid[c] == UNK and self.evid[c] < WALL_EVID:
            self.grid[c] = FREE
        elif self.grid[c] == WALL and self.evid[c] <= 2:
            self.grid[c] = FREE

    def sense(self, bx, by, yawr, rmin, rmax, ok, planted, tall=True):
        """rmin/rmax: per-bearing rolling window extremes; ok[b]: window full
        (collected while tracking straight). planted: standing truly still,
        knees below the ring plane -> the ambiguous shell is believable.
        tall: at cruise height (not crouched) -> see-through counts against
        LOW cells (the doubt that purges stale crouch-lane breadcrumbs)."""
        mx = bx - math.sin(yawr) * MAST_FWD
        my = by + math.cos(yawr) * MAST_FWD
        # standing on it = strongest evidence it's free (rays can't see under himself)
        hc = cell_of(bx, by)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                c = (hc[0]+dx, hc[1]+dy)
                if in_grid(c) and self.grid[c] not in (WALL, LOW) \
                        and self.evid[c] < FRONTIER_EVID:
                    self.grid[c] = FREE       # don't resurrect wall-evidence cells
        for b in range(24):
            if not ok[b]:
                continue
            az = math.radians(15 * b)
            dx, dy = math.sin(az - yawr), math.cos(az - yawr)
            if rmax[b] >= SELF_MIN:
                # carve free to the window MAX: knee blips only ever SHORTEN a
                # return, so the longest sample in the window is the true ray —
                # this is what lets him map BEHIND himself while walking
                reach = min(rmax[b], RAY_MAX) - 0.06
                n = int(reach / (CS * 0.5))
                for k in range(1, n + 1):
                    d = k * CS * 0.5
                    px, py = mx + dx * d, my + dy * d
                    c = cell_of(px, py)
                    if not in_grid(c):
                        break
                    # only if the ray passes near the CELL CENTER: a wall face
                    # rarely sits on a cell boundary, so mixed wall/corridor
                    # cells get grazed along their free strip — clipping a
                    # corner must not erase the wall evidence (the phantom
                    # doorways that kept luring him into wall faces)
                    cwx, cwy = world_of(c)
                    if abs(px - cwx) < 0.0475 and abs(py - cwy) < 0.0475:
                        self._see_through(c, tall)
            if rmax[b] < RAY_MAX and rmax[b] >= SELF_HARD:
                c = cell_of(mx + dx * rmax[b], my + dy * rmax[b])
                if in_grid(c):
                    if rmax[b] >= SELF_MIN:
                        self._wall_evidence(c, 4)
                        # WALL SHADOW: push evidence one step past the hit, so
                        # the cell behind a struck face loses UNKNOWN status —
                        # a frontier can then never live on a wall face (the
                        # phantom-doorway lure), while real doorways (no
                        # endpoint) are untouched. If it's actually free space
                        # seen later from the far side, see-through erodes it.
                        for step in (0.075, 0.15):
                            cb = cell_of(mx + dx*(rmax[b]+step), my + dy*(rmax[b]+step))
                            if in_grid(cb) and cb != c:
                                # only shadow the UNKNOWN: the shadow's job is
                                # killing phantom frontiers behind walls — on
                                # known-free space it slowly ate one column of
                                # every corridor (seen from the far side)
                                if self.grid[cb] == UNK:
                                    self._wall_evidence(cb, 2)
                                break
                    elif planted:
                        # 0.11-0.16 shell: knees live here while stepping, but
                        # planted + persistent for the whole window = real wall
                        self._wall_evidence(c, 1)

    # ---------------- planning ----------------
    def _blocks(self, c):
        # CONFIRMED walls only: letting transient bump marks count here made
        # every contact block radiate — neighbors read "one cell wide" and
        # corridors sealed shut in a chain reaction (bump already blocks its
        # own cell in passable(), which is all a contact mark has earned)
        if not in_grid(c):
            return False        # beyond the grid is open floor, not a wall
        return self.grid[c] == WALL

    def passable(self, c):
        # wall evidence vetoes routing even while the state flickers FREE —
        # mixed wall/corridor cells are walkable on the map, not in the world.
        # LOW is exempt: a gimbal verdict deliberately overrides wall evidence
        # (the crouch lane RUNS THROUGH the lintel's wall cells)
        if self.bump[c] >= 8:
            return False
        if self.grid[c] == LOW:
            return True
        return self.grid[c] == FREE and self.evid[c] < 6

    def narrow(self, c):
        """Map says this free cell is walled on both opposite sides — a gap of
        at most one cell. At 15 cm resolution this CANNOT distinguish the
        impossible 0.13 m slot from the walkable 0.43 m two-row lane one of
        whose rows got scatter-walled — so it must never ban routing (that
        sealed the over-C lane and killed exploration). It is a SUSPICION:
        combined with a physical stuck it becomes proof — "I don't fit"."""
        i, j = c
        return (self._blocks((i-1, j)) and self._blocks((i+1, j))) or                (self._blocks((i, j-1)) and self._blocks((i, j+1)))

    def _near_wall_mask(self):
        w = (self.grid == WALL)
        m = np.zeros_like(w)
        m[1:, :] |= w[:-1, :]; m[:-1, :] |= w[1:, :]
        m[:, 1:] |= w[:, :-1]; m[:, :-1] |= w[:, 1:]
        m[1:, 1:] |= w[:-1, :-1]; m[:-1, :-1] |= w[1:, 1:]
        m[1:, :-1] |= w[:-1, 1:]; m[:-1, 1:] |= w[1:, :-1]
        return m

    def dijkstra(self, start):
        """Cost field over passable cells; wall-adjacent cells cost extra so
        routes prefer corridor centers. Returns (dist, prev, root) where root
        maps each cell to the FIRST STEP taken from start toward it."""
        nw = self._near_wall_mask()
        inc = set(self.path)     # incumbent route: small discount so the
        # standing plan wins cost TIES — two equal routes otherwise flip
        # winner with every evidence wobble and oscillate him to a standstill
        dist = {start: 0.0}
        prev = {start: None}
        root = {start: None}
        pq = [(0.0, start)]
        while pq:
            d, c = heapq.heappop(pq)
            if d > dist.get(c, 1e18):
                continue
            for dx, dy in N4:
                nc = (c[0]+dx, c[1]+dy)
                if not in_grid(nc) or not self.passable(nc):
                    continue
                nd = d + 1.0 + (1.5 if nw[nc] else 0.0)                        + (self.low_extra if self.grid[nc] == LOW else 0.0)                        - (0.2 if nc in inc else 0.0)
                if nd < dist.get(nc, 1e18):
                    dist[nc] = nd
                    prev[nc] = c
                    root[nc] = nc if c == start else root[c]
                    heapq.heappush(pq, (nd, nc))
        return dist, prev, root

    def _trace(self, prev, c):
        path = [c]
        while prev[path[-1]] is not None:
            path.append(prev[path[-1]])
        return path[::-1]

    def unk_neighbors(self, c):
        return sum(1 for dx, dy in N4
                   if in_grid((c[0]+dx, c[1]+dy)) and self.grid[(c[0]+dx, c[1]+dy)] == UNK)

    def plan_frontier(self, start, bx, by, yawr, t):
        """Score every reachable frontier; pick the best, not the whipsaw-nearest.
        Sets self.goal and returns the path, or None if the map is exhausted.
        Bans are a soft veto: if they eliminated EVERY candidate, forgive the
        oldest and retry — a scan should only fire when the map itself says
        there is nowhere left, not when motion hiccups banned everything."""
        dist, prev, root = self.dijkstra(start)
        for honor_bans in (True, False):
            best, best_s = None, 1e18
            for c, d in dist.items():
                if abs(c[0]-start[0]) + abs(c[1]-start[1]) < 2:
                    continue                       # a frontier is somewhere ELSE
                if honor_bans and self.banned.get(c, -1.0) > t:
                    continue
                if self.evid[c] >= FRONTIER_EVID:
                    continue       # keeps taking endpoint hits = a wall face, not a doorway
                nunk = self.unk_neighbors(c)
                if nunk == 0:
                    continue
                fs = root[c]
                wx, wy = world_of(fs if fs is not None else c)
                turn = abs(wrap(yawr - math.atan2(-(wx-bx), wy-by))) / math.pi
                s = d + 6.0*turn - 0.4*nunk
                if not honor_bans:
                    if self.banned.get(c, -1e18) > t + BAN_S - 20.0:
                        continue                    # banned <20 s ago: not yet forgiven
                    s = self.banned.get(c, -1e18)   # retry the least-recently-banned
                if s < best_s:
                    best_s, best = s, c
            if best is not None:
                if not honor_bans:
                    self.banned.pop(best, None)
                self.goal = best
                return self._trace(prev, best)
        self.goal = None
        return None

    def plan_scan_spot(self, start, scanned_pts, thorough=False):
        """Map exhausted: route to a standpoint next to a wall with UNKNOWN
        beyond it (a wall he can't explain), skipping spots already scanned.
        The gimbal fingerprint only works up close — so go stand there.
        thorough (survey mode): EVERY wall face he can stand near counts,
        known-beyond or not — low tunnels are invisible to the standing ring
        by design, so a real survey interrogates every wall."""
        # suspect walls: WALL cells with unknown 2-3 steps beyond (walls are
        # up to 2 cells thick with shadow evidence) = walls he can't explain
        suspects = []
        for i in range(GNX):
            for j in range(GNY):
                if self.grid[i, j] != WALL:
                    continue
                wx0, wy0 = world_of((i, j))
                if thorough and not (SURVEY_BOUNDS[0] < wx0 < SURVEY_BOUNDS[1]
                                     and SURVEY_BOUNDS[2] < wy0 < SURVEY_BOUNDS[3]):
                    continue
                for dx, dy in N4:
                    if thorough:
                        n = (i+dx, j+dy)
                        if in_grid(n) and self.passable(n):
                            suspects.append(world_of((i, j)))
                            break
                        continue
                    b2, b3 = (i+2*dx, j+2*dy), (i+3*dx, j+3*dy)
                    if (in_grid(b2) and self.grid[b2] == UNK) or \
                       (in_grid(b3) and self.grid[b3] == UNK):
                        suspects.append(world_of((i, j)))
                        break
        # a suspect within 0.35 m of a past scan HAS been interrogated — the
        # verdict range is short (dipped ray physics), so a wide dedup radius
        # was skipping the winning standpoint as "already covered"
        # dedup radius (0.38) is deliberately LARGER than the coverage radius
        # (0.33): a completed scan must retire every suspect its standpoint
        # claimed, or edge jitter re-picks the same spot forever
        open_sus = [(sx, sy) for sx, sy in suspects
                    if not any((sx-px)**2 + (sy-py)**2 < 0.38**2
                               for px, py in scanned_pts)]
        if not open_sus:
            return None
        dist, prev, _ = self.dijkstra(start)
        best, best_s = None, 1e18
        for c, d in dist.items():
            wx, wy = world_of(c)
            if thorough and not (SURVEY_BOUNDS[0] < wx < SURVEY_BOUNDS[1]
                                 and SURVEY_BOUNDS[2] < wy < SURVEY_BOUNDS[3]):
                continue    # stand INSIDE the arena; inside scans retire
                            # outer-wall cells too — no perimeter tour
            cover = sum(1 for sx, sy in open_sus
                        if (sx-wx)**2 + (sy-wy)**2 < 0.33**2)
            if cover == 0:
                continue
            s = d - 4.0*cover           # near spots that interrogate MORE wall win
            if s < best_s:
                best_s, best = s, c
        return self._trace(prev, best) if best is not None else None

    def plan_to(self, start, goal):
        dist, prev, _ = self.dijkstra(start)
        return self._trace(prev, goal) if goal in dist else None

    def plan_exit(self, start):
        """RACE MODE: he already knows the maze — route straight for the
        finish (any passable cell beyond the east wall)."""
        dist, prev, _ = self.dijkstra(start)
        best, best_d = None, 1e18
        for c, d in dist.items():
            if world_of(c)[0] >= FINISH_X - 0.05 and d < best_d:
                best_d, best = d, c
        if best is None:
            return None
        path = self._trace(prev, best)
        if len(path) == 1:
            # already standing on a finish cell — keep walking east, don't
            # report "no route" (that read as an impasse and fired DOUBT
            # three centimeters from the line)
            nxt = (best[0] + 1, best[1])
            if in_grid(nxt):
                path.append(nxt)
        return path

    def goal_valid(self, t):
        """Is the committed goal still worth chasing? (still a frontier,
        not banned, not turned out to be a wall face)"""
        g = self.goal
        return (g is not None and in_grid(g)
                and self.banned.get(g, -1.0) < t
                and self.evid[g] < FRONTIER_EVID
                and self.passable(g)
                and self.unk_neighbors(g) > 0)

    def carve_low(self, mx, my, yawr, az, max_d=1.0, start_d=None, edge=None):
        """Gimbal tunnel verdict: mark cells along the bearing as LOW (crouch
        lane). The carve is a HYPOTHESIS — remembered so a failed traverse
        can revoke it (revoke_carves). start_d anchors the lane at the WALL
        FACE, not the scan spot — painting from 0.1 m marked up to 70 cm of
        open approach floor as crouch-lane (Bill: "low crawling towards it
        from a mile away")."""
        dx, dy = math.sin(az - yawr), math.cos(az - yawr)
        if start_d is None:
            start_d = max(0.1, self.expected_range(mx, my, dx, dy) - 0.15)
        # the LANE is the line through the wall, not the scan ray: lay the
        # cell trail along the SNAPPED AXIS from the anchor. A diagonal ray
        # trail put lane cells in the wrong grid row — his body threaded the
        # row below, here==LOW never refreshed the crouch, and he stood up
        # under the lintel (the 150 s standing-wedge at the mouth).
        ax0, ay0 = ((math.copysign(1.0, dx), 0.0) if abs(dx) > abs(dy)
                    else (0.0, math.copysign(1.0, dy)))
        anx, any_ = mx + dx * (start_d + 0.15), my + dy * (start_d + 0.15)
        for gi, g in enumerate(self.carve_geo):
            if (g[2]*ax0 + g[3]*ay0 > 0.9
                    and abs((anx-g[0])*g[2] + (any_-g[1])*g[3]) < 0.20
                    and abs(-(anx-g[0])*g[3] + (any_-g[1])*g[2]) < 0.18):
                # SAME opening re-scanned (mid-race peeks re-verdict a mouth
                # they stumble at): a second geo on one opening gives geo_for
                # two masters — the line target alternated and he pivoted in
                # place for 100 s. Keep the incumbent line and its bracket.
                return gi
        cells = []
        d = -0.15
        while d < max_d - 0.15:
            c = cell_of(anx + ax0 * d, any_ + ay0 * d)
            if in_grid(c):
                self.grid[c] = LOW
                cells.append(c)
            d += CS * 0.5
        self.carves.append(cells)
        # keep the MEASURED centerline (sub-cell): the scan's span-weighted
        # bearing crosses the wall plane at the opening's true center; snap
        # the lane axis to the wall normal (walls are axis-aligned here — the
        # firmware port can carry the raw bearing). lane_center projects onto
        # this line instead of re-deriving 15 cm cell-center geometry.
        self.carve_geo.append((anx, any_, ax0, ay0))
        self.carve_edge.append(edge if edge is not None
                               else [1e9, -1e9, -1e9, 1e9, 0, 0, 0])
        return len(self.carve_geo) - 1

    @staticmethod
    def edge_accum(edge, P, axy, mx, my, dirx, diry, r):
        """Fold one crouched ray into an opening-edge bracket. P/axy: a point
        on the lane line and its unit axis. Flank HITS (endpoint near the wall
        plane, off-axis) bound the opening from outside; PASS-THROUGHS (ray
        continues well past the plane) bound it from inside."""
        ax_, ay_ = axy
        nx_, ny_ = -ay_, ax_
        den = dirx*ax_ + diry*ay_
        if den < 0.3:
            return
        d_wall = -((mx-P[0])*ax_ + (my-P[1])*ay_) / den
        if d_wall < 0.05 or d_wall > 1.0:
            return
        r_eff = min(r, RAY_MAX - 0.01)
        if r_eff > d_wall + 0.10:                      # passed the plane
            if den < 0.85:
                return
            # near-axis passes only: the crossing is evaluated at the ANCHOR
            # plane (up to ~6 cm off the true wall), so a diagonal ray's
            # cross is mis-attributed by tan(angle) * that gap — one such
            # sample put a "pass" through solid wall and broke the bracket
            cx = (mx + dirx*d_wall - P[0])*nx_ + (my + diry*d_wall - P[1])*ny_
            if abs(cx) < 0.30:
                edge[2] = max(edge[2], cx)
                edge[3] = min(edge[3], cx)
                edge[6] += 1
        elif 0.22 <= r <= 0.50 and abs(r - d_wall) <= 0.14:  # endpoint on the plane
            # r floor 0.22: his own PLANTED knee-tops return persistently at
            # up to ~0.19 from the mast (window max can't erase a leg that
            # holds still) — real flank hits from the micro-peek are >=0.3
            # hits only from SHORT range: within 0.5 m the level gate
            # geometrically excludes lintel/floor clips (a 3.4 deg ray needs
            # >0.5 m to climb the 36 mm from crouched ring to lintel bottom);
            # long slightly-pitched rays were landing fake "flank hits"
            # INSIDE the opening span and collapsing the bracket.
            # hit gate wider (0.38) than the pass gate: a first line biased
            # toward one edge must still see the FAR flank to get corrected
            cx = (mx + dirx*r - P[0])*nx_ + (my + diry*r - P[1])*ny_
            if 0.02 < cx < 0.38:
                edge[0] = min(edge[0], cx)
                edge[4] += 1
            elif -0.38 < cx < -0.02:
                edge[1] = max(edge[1], cx)
                edge[5] += 1

    def edge_offset(self, gi):
        """Refined center offset (along the lane normal) for carve gi, or 0
        until the bracket has enough evidence and passes sanity."""
        e = self.carve_edge[gi]
        # endpoint hits are PHYSICAL (a point on the actual face) — when both
        # flanks have real hit counts and the width is plausible, trust them
        # outright; passes only bound the estimate when hits are sparse
        if e[4] >= 3 and e[5] >= 3 and 0.24 < e[0] - e[1] < 0.44:
            return max(-0.12, min(0.12, (e[0] + e[1]) / 2.0))
        # blended fallback, PER-SIDE convergence-gated: the true edge lies
        # between the innermost hit and the outermost pass, so a side only
        # counts when that interval is tight. A wild-geometry survey admitted
        # a far-wall-face hit at cross 0.29 (gate is 0.38) with no closer
        # sample — the old count-only gate blended it into "offset ~0" and
        # the traverse rode the wall again.
        n_ok = e[4] >= 2 and e[6] >= 1 and (e[0] - e[2]) <= 0.15
        s_ok = e[5] >= 2 and e[6] >= 1 and (e[3] - e[1]) <= 0.15
        if n_ok and s_ok and e[0] > e[2] - 0.03 and e[1] < e[3] + 0.03:
            hi = (e[0] + e[2]) / 2.0
            lo = (e[1] + e[3]) / 2.0
            if 0.20 < hi - lo < 0.48:
                return max(-0.12, min(0.12, (hi + lo) / 2.0))
        # ONE side converged: center via the half-span prior (0.17). From a
        # biased line the near-axis pass fan geometrically cannot reach the
        # far edge (den>=0.85 subtends ~+/-0.21 at typical peek range), so
        # the far side may NEVER converge — one good edge still beats
        # offset 0, and the +/-0.12 clamp bounds a wrong-span world.
        if n_ok and not s_ok:
            return max(-0.12, min(0.12, (e[0] + e[2]) / 2.0 - 0.17))
        if s_ok and not n_ok:
            return max(-0.12, min(0.12, (e[1] + e[3]) / 2.0 + 0.17))
        return 0.0

    def bracket_ok(self, gi):
        """Is this carve's edge bracket GOOD (converged both sides)? The
        micro-peek fires until this holds (or attempts run out) — count-only
        completeness let sampling-luck brackets suppress the head-on look."""
        e = self.carve_edge[gi]
        if e[4] >= 3 and e[5] >= 3 and 0.24 < e[0] - e[1] < 0.44:
            return True
        return (e[4] >= 2 and e[6] >= 1 and (e[0] - e[2]) <= 0.15
                and e[5] >= 2 and (e[3] - e[1]) <= 0.15)

    def geo_for(self, wx, wy):
        """The carve line governing a world point (latest wins), as
        (key, index), or None. Key is index-stable across revokes."""
        for gi in range(len(self.carve_geo) - 1, -1, -1):
            g = self.carve_geo[gi]
            along = (wx-g[0])*g[2] + (wy-g[1])*g[3]
            cross = abs((wx-g[0])*g[3] - (wy-g[1])*g[2])
            if cross < 0.16 and -0.30 < along < 1.20:
                return (round(g[0], 2), round(g[1], 2)), gi
        return None

    def on_lane(self, bx, by):
        """Body is in a lane's crouch envelope — LINE-based, so the row the
        trail cells landed in doesn't decide whether he stays down."""
        for g in self.carve_geo:
            along = (bx-g[0])*g[2] + (by-g[1])*g[3]
            cross = abs((bx-g[0])*g[3] - (by-g[1])*g[2])
            # generous cross: the line may start up to ~0.11 biased before
            # the edge bracket corrects it — standing up mid-mouth because
            # the body left a mis-centered envelope was a lintel wedge.
            # along ends at +0.42: body fully clear of the wall plane +
            # margin — crouching 0.85 m past the anchor was pure knee bill
            if cross < 0.17 and -0.18 < along < 0.42:
                return True
        return False

    def refine_crouched(self, mx, my, yawr, wmax):
        """While crouched (ring under the lintel) fold the ring into every
        nearby carve's bracket. Uses the WINDOW MAX, not the raw ring: a
        crouched wave gait blips the ring constantly (knees near the low
        plane) and raw blips masked every flank hit (bracket 0/0 after a
        full traverse). At crouch speed the 60 ms window smears ~2 mm."""
        for gi, g in enumerate(self.carve_geo):
            if (mx-g[0])**2 + (my-g[1])**2 > 0.8**2:
                continue
            for b in range(24):
                r = wmax[b]
                if r is None or r < SELF_MIN:
                    continue
                azb = math.radians(15*b)
                self.edge_accum(self.carve_edge[gi], (g[0], g[1]), (g[2], g[3]),
                                mx, my, math.sin(azb - yawr), math.cos(azb - yawr), r)

    def revoke_carves(self, bx, by, r=0.40):
        """He physically stuck on a carved lane: the verdict was wrong.
        Revert nearby LOW cells to unknown — wall evidence reasserts itself.
        Works without provenance (a LOADED mind has no carve list): falls back
        to reverting the contiguous LOW cluster around the stuck point."""
        hit = False
        for cells in self.carves:
            if any((world_of(c)[0]-bx)**2 + (world_of(c)[1]-by)**2 < r*r
                   for c in cells):
                for c in cells:
                    if self.grid[c] == LOW:
                        self.grid[c] = UNK
                hit = True
        # drop centerlines whose line passes near the revoked spot — a later
        # re-carve must not have its lane projected onto a dead verdict's line
        keep = [k for k, g in enumerate(self.carve_geo)
                if abs((bx-g[0])*g[3] - (by-g[1])*g[2]) > 0.20
                or abs((bx-g[0])*g[2] + (by-g[1])*g[3]) > 0.90]
        self.carve_geo = [self.carve_geo[k] for k in keep]
        self.carve_edge = [self.carve_edge[k] for k in keep]
        if hit:
            self.carves = [cs for cs in self.carves
                           if not all(self.grid[c] != LOW for c in cs)]
            return True
        seeds = [(i, j) for i in range(GNX) for j in range(GNY)
                 if self.grid[i, j] == LOW
                 and (world_of((i, j))[0]-bx)**2 + (world_of((i, j))[1]-by)**2 < r*r]
        if not seeds:
            return False
        stack = list(seeds)
        while stack:
            c = stack.pop()
            if self.grid[c] != LOW:
                continue
            self.grid[c] = UNK
            for dx, dy in N4:
                nc = (c[0]+dx, c[1]+dy)
                if in_grid(nc) and self.grid[nc] == LOW:
                    stack.append(nc)
        return True

    def lane_center(self, c):
        """A crouch-lane cell's TRUE cross-position. Best source: the carve's
        MEASURED centerline (the scan located the opening sub-cell; project
        the cell onto that line). Fallback for legacy minds: midpoint between
        flanking WALL cells — but at 15 cm cells neither grid row centers a
        0.34 m tunnel and wall CELL CENTERS put the line at 0.725 vs the true
        0.70 (Bill: left middle foot dragging the north wall while the trace
        bragged about body clearance — the 2-4 cm audit residual)."""
        wx, wy = world_of(c)
        best, best_cross = None, 0.16
        for gi in range(len(self.carve_geo) - 1, -1, -1):   # latest verdict wins
            g = self.carve_geo[gi]
            along = (wx-g[0])*g[2] + (wy-g[1])*g[3]
            cross = abs((wx-g[0])*g[3] - (wy-g[1])*g[2])
            if cross < best_cross and -0.30 < along < 1.20:
                off = self.edge_offset(gi)
                best_cross = cross
                best = (g[0] + g[2]*along - g[3]*off,
                        g[1] + g[3]*along + g[2]*off)
        if best is not None:
            return best
        i, j = c
        for axis in (0, 1):
            lo = hi = None
            for k in (1, 2):
                cc = (i - k, j) if axis == 0 else (i, j - k)
                if lo is None and in_grid(cc) and self.grid[cc] == WALL:
                    lo = world_of(cc)[axis]
                cc = (i + k, j) if axis == 0 else (i, j + k)
                if hi is None and in_grid(cc) and self.grid[cc] == WALL:
                    hi = world_of(cc)[axis]
            if lo is not None and hi is not None:
                mid = (lo + hi) / 2.0
                if axis == 0:
                    wx = mid
                else:
                    wy = mid
        return wx, wy

    def waypoint(self, c):
        """Cell -> world target, nudged away from adjacent walls. His stance
        is 0.29 m wide: aiming at the raw center of a lane cell parks his
        outer feet inside the wall corner (the over-A wedge, 300 s of it)."""
        wx, wy = world_of(c)
        for dx, dy in N4:
            n = (c[0]+dx, c[1]+dy)
            o = (c[0]-dx, c[1]-dy)
            if in_grid(n) and self.grid[n] == WALL and \
               in_grid(o) and self.grid[o] != WALL:
                wx -= dx*0.075
                wy -= dy*0.075
        return wx, wy

    def los(self, x0, y0, x1, y1, width=0.0):
        """Straight line stays in passable cells of MY OWN map?
        width > 0 = FAT line-of-sight: also check a parallel line at each
        shoulder. A zero-width line that clears a corner by 1 cm is "clear"
        for a point but not for his 0.29 m stance — the corner-cutting snag
        (Bill's callout: hurrying the cut cost him more than the wide line)."""
        d = math.hypot(x1-x0, y1-y0)
        if d < 1e-6:
            return True
        px, py = -(y1-y0)/d, (x1-x0)/d      # unit perpendicular
        offsets = (0.0,) if width <= 0 else (-width, 0.0, width)
        n = max(2, int(d / 0.05))
        for off in offsets:
            ox, oy = px*off, py*off
            for k in range(1, n + 1):
                f = k / n
                c = cell_of(x0 + (x1-x0)*f + ox, y0 + (y1-y0)*f + oy)
                if not in_grid(c) or not self.passable(c):
                    return False
        return True

    def expected_range(self, mx, my, wdir_x, wdir_y, max_d=1.2):
        """What the MAP says a level ray should measure along a world
        direction: distance to the first believed-WALL cell."""
        d = 0.05
        while d < max_d:
            c = cell_of(mx + wdir_x*d, my + wdir_y*d)
            if not in_grid(c):
                return max_d
            if self.grid[c] == WALL:
                return d
            d += 0.05
        return max_d

    def coverage(self):
        return int((self.grid != UNK).sum())


# THE DOOR SCENARIO (Bill 2026-08-21: "lets set him up with the door maze
# again... change the layout of the maze entirely... add a couple doors.
# let him solve it, then change if the doors are open or closed, then see
# which ones he tries etc. maybe in one instance, lock one of the doors").
# Shared by the headless CLI (--scenario doors) and watch_nav (his windows).
DOOR_SCENARIO = [
    ("RUN 1 -- first contact: D1 shut(t1), D2 shut(t2), D3 open",
     dict(maze3=True, door_cfg=dict(nvd1=dict(resist=0.02),
                                    nvd2=dict(resist=0.25),
                                    nvd3=dict(open=True))),
     dict(save_mind="out/mind_vault.npz", max_s=560)),
    ("RUN 2 -- the switch: D1 open, D2 shut(t1), D3 shut(t2)",
     dict(maze3=True, door_cfg=dict(nvd1=dict(open=True),
                                    nvd2=dict(resist=0.02),
                                    nvd3=dict(resist=0.25))),
     dict(race=True, mind_path="out/mind_vault.npz",
          save_mind="out/mind_vault.npz", max_s=420)),
    ("RUN 3 -- the lock: D1 open, D2 LOCKED, D3 open",
     dict(maze3=True, door_cfg=dict(nvd1=dict(open=True),
                                    nvd2=dict(resist=0.25, locked=True),
                                    nvd3=dict(open=True))),
     dict(race=True, mind_path="out/mind_vault.npz",
          save_mind="out/mind_vault_r3.npz", max_s=620)),
    # (620 s: RUN 3 must sweep, breach, LOSE to a locked door, perform
    #  the tragedy, re-examine the wall, discover the crouch tunnel and
    #  crawl it -- the scenario's longest story by design)
]


def run(view=False, video=None, verbose=True, dbg=False, map_cb=None, tune=None,
        world_kw=None, race=False, mind_path=None, save_mind=None, survey=False,
        shifts=None, max_s=None, start=None, frame_cb=None, cam_ctl=None):
    """map_cb, if given, is called ~2x/sim-second with a snapshot of his MIND:
    dict(grid, path, goal, x, y, yaw, t, mode, scans, coverage) — the hook
    watch_nav.py uses to draw the live map beside the MuJoCo window.
    tune overrides TUNE genes (time-trial evolution).
    world_kw -> extra build_xml flags (e.g. shortcut=True).
    race: load the mind from mind_path and route straight for the finish.
    survey: don't stop at the finish line — explore + scan until no
    unexplained wall remains (builds the complete map for race mode).
    save_mind: path to persist the map at the end of the run.
    shifts: SHIFTING LABYRINTH — list of dicts(name, when(t,x,y)->bool,
    apply(model)); each fires once, mid-run, mutating the world geometry."""
    tn = {**TUNE, **(tune or {})}
    cr = dict(CRUISE, speed=tn["cruise"], lift=tn["lift"])
    dk = dict(DUCK, speed=tn["duck"])
    model = mujoco.MjModel.from_xml_string(G.build_xml(
        maze=True, **{k: v for k, v in (world_kw or {}).items()
                      if k not in ("door_open", "door_cfg")}))
    data = mujoco.MjData(model)
    if (world_kw or {}).get("door_open"):
        _dj = model.joint("door_hinge")
        data.qpos[int(_dj.qposadr[0])] = math.radians(95.0)
        # the door starts OPEN — survey runs map the passage FREE, so
        # a later shut-door race has the memory Bill specified: "he's
        # been through the passage before, but now the way is blocked"
    door_reg = []
    if (world_kw or {}).get("maze3"):
        # THE VAULT's tiered doors (Bill: "tier 1, 2, 3 doors... in one
        # instance, lock one of the doors"). Registry: hinge address +
        # the face-door heading for his approach side; door_cfg sets
        # per-door resistance / lock / open state at runtime.
        for _dn, _fx, _fy, _hd, _osgn, _hx, _hy, _pdx, _pdy in (
                ("nvd1", -0.04, 0.315, 0.0, 1.0, -0.225, 0.30, 1.0, 0.0),
                ("nvd2", 1.115, 0.91, -math.pi / 2, -1.0, 1.10, 0.705, 0.0, 1.0),
                ("nvd3", 2.315, 0.69, -math.pi / 2, -1.0, 2.30, 0.485, 0.0, 1.0)):
            _j = model.joint(_dn + "_hinge")
            door_reg.append(dict(name=_dn, adr=int(_j.qposadr[0]),
                                 dof=int(_j.dofadr[0]), jid=int(_j.id),
                                 x=_fx, y=_fy, hd=_hd, osgn=_osgn,
                                 hx=_hx, hy=_hy, pdx=_pdx, pdy=_pdy))
        for _dn, _cfg in (((world_kw or {}).get("door_cfg")) or {}).items():
            _e = next(d for d in door_reg if d["name"] == _dn)
            model.dof_frictionloss[_e["dof"]] = _cfg.get("resist", 0.02)
            if _cfg.get("locked"):
                # a REAL lock: friction can't stop a connected shove
                # (the gym sweep), only a blocked hinge can
                model.jnt_range[_e["jid"]] = (-0.035, 0.035)
            if _cfg.get("open"):
                data.qpos[_e["adr"]] = _e["osgn"] * math.radians(
                    _cfg.get("open_deg", 92.0))
    rock_adr = rock_dof = None
    if (world_kw or {}).get("rocker"):
        _j = model.joint("rock_hinge")
        rock_adr, rock_dof = int(_j.qposadr[0]), int(_j.dofadr[0])
    scan3d_new = []            # 3D endpoints his scans painted since last emit
    _rf3d = []                 # (site id, sensor addr) for every rangefinder
    for _nm in ([f"ld{i}" for i in range(24)] + [f"nod{i}" for i in range(5)]
                + [f"high{i}" for i in range(3)]):
        try:
            _rf3d.append((model.site(_nm).id,
                          int(model.sensor(f"rf_{_nm}").adr[0])))
        except Exception:
            pass
    robot_bodies = set()
    _stack = [model.body("base").id]
    while _stack:                      # base + every descendant link
        _bid = _stack.pop()
        robot_bodies.add(_bid)
        for _k in range(model.nbody):
            if int(model.body_parentid[_k]) == _bid and _k not in robot_bodies:
                _stack.append(_k)
    _badr = int(model.jnt_qposadr[model.body("base").jntadr[0]])
    # freejoint address resolved, not assumed: the rocker world's hinge
    # precedes the robot in qpos and silently shifted these writes
    data.qpos[_badr + 2] = G.COXA_Z
    if start is not None:               # (x, y, yaw) — test-harness spawns
        data.qpos[_badr + 0], data.qpos[_badr + 1] = start[0], start[1]
        data.qpos[_badr + 3] = math.cos(start[2] / 2.0)
        data.qpos[_badr + 6] = math.sin(start[2] / 2.0)
    drv = G.GaitDriver("TRIPOD", 1.0, lift_cm=2.15, cadence_ms=tn["cadence"],
                       speed_cap=1.0, touchdown="match")
    drv.v = 0.0                               # stand while the map warms up
    mind = Mind.load(mind_path) if (race and mind_path) else Mind()
    mind.low_extra = tn["low_extra"]
    heading = None
    scout = None
    scan_next = False          # walking to a scan standpoint
    scanned_pts = []
    scan_cool = -1.0
    crouch_until = -1.0
    measure_until = -1.0
    mouth_measured = {}        # geo key -> measuring attempts (max 2)
    back_until = -1.0
    pivot_dir = 0.0
    pivot_since = None
    replan_at = 0.0
    idle_until = -1.0
    bump_decay_at = 10.0
    goal_since = 0.0
    goal_best_d = 10**9        # best path length seen toward the committed goal
    goal_progress_t = 0.0
    stuck_hist = deque()       # (t, x, y) for the no-displacement detector
    last_move_xy = None        # deadman: last spot he actually moved from
    last_move_t = 0.0
    posbuf = deque(maxlen=60)  # ~0.6 s of positions (the "actually moving" gate)
    lane_int = 0.0             # cross-track integral for lane line-following
    att_fp = att_fr = 0.0      # walking attitude servo: filtered IMU
    att_ip = att_ir = 0.0      # ... integrators
    att_on, att_arm = False, 0  # ... hysteresis arming state
    slope_mp = slope_mr = slope_ma = 0.0   # hill-vs-deck persistence
    com_ip2 = com_ir2 = 0.0    # ... COM companion integrators
    com_prev, com_rate = [0.0, 0.0], 0.0
    door_cool = -1.0           # shut-door ladder cooldown
    door_next = None           # walking to a shut-door suspect cell
    door_next_t = 0.0          # when the commute began (stale release)
    door_unreachable = set()   # suspects whose commute never arrived --
    # release WITHOUT memory is just a slower carousel (he re-picked the
    # same unreachable cell every 40 s for a whole run)
    door_tried_at = {}         # per-door curiosity cooldown
    door_visits = {}           # per-door visit count (hard cap)
    deadman_hist = []          # recent deadman firings -> escalate
    untangle_until = -1.0      # THE UNTANGLE REFLEX (frame-verified
    untangle_ph = 0            # wedge: HIGH-CENTERED on a swung door
    untangle_side = 1.0        # panel, belly on the edge, marching in
    untangle_n = 0             # place -- normal back-off can't free it)
    surge_until = -1.0         # post-breach FOLLOW-THROUGH: drive out
    surge_hd = 0.0             # the door normal before replanning --
    # a fresh breach leaves him IN the mouth surrounded by unresolved
    # frontier confetti; goal selection orbited each nearby cell in turn
    push_wide = None           # clear-bin count when the push began
    push_ref = None            # (x, y, front) -- the PUSHING detector
    push_until = -1.0          # (Bill: "he should realize when he's
    # pushing objects... he's moving, but it's staying the same
    # distance" -- constant range + accumulating odometry = a MOVABLE,
    # not a wall; his pivot was flip-flopping against a yielding panel)
    door_done_names = set()    # conquered doors leave the ladder pool
    for _dr0 in door_reg:
        if abs(math.degrees(float(data.qpos[_dr0["adr"]]))) >= 72.0:
            # a door standing OPEN at run start was never an adversary:
            # sweep-only forever, no mounts, no tragedy (runs 2/3 kept
            # performing the full defeat on their own brushed-shut
            # entry doors)
            door_done_names.add(_dr0["name"])
    door_done = set()          # doors already breached open this run --
    # their WALL-remembered gap cells must leave the suspect pool, or he
    # walks back and re-ladders a door he opened (RUN 2 breached nvd2
    # three times in a row)
    stag_cov = -1              # coverage-stagnation escape: frozen
    stag_t = 0.0               # coverage = trapped in a local carousel
    stag_leave_until = -1.0    # (a false-lane scan/carve/revoke loop
    #                            held him 200+ s at 310 cells)
    door_painted = {}          # per-door cells we painted last sync

    def sync_doors():
        # DOORS ARE MAP OBJECTS (the architectural fix after three
        # sessions of whack-a-mole): the mind tracks each registered
        # door's TRUE pose -- panel cells WALL where the panel IS, gap
        # cells FREE when it stands open, gap bumps cleared. September:
        # this models line-fitting the panel in the lidar scan.
        for _d in door_reg:
            _q = float(data.qpos[_d["adr"]])
            _deg = abs(math.degrees(_q))
            _cq, _sq = math.cos(_q), math.sin(_q)
            _pdx = _d["pdx"] * _cq - _d["pdy"] * _sq
            _pdy = _d["pdx"] * _sq + _d["pdy"] * _cq
            # un-paint last sync's cells (neutral: let sensors re-own)
            for _c in door_painted.get(_d["name"], ()):
                if in_grid(_c) and mind.grid[_c] == WALL:
                    mind.grid[_c] = UNK
                    mind.evid[_c] = 0
            _newly = set()
            # the panel, where it actually is
            _s = 0.04
            while _s < 0.38:
                _c = cell_of(_d["hx"] + _pdx * _s, _d["hy"] + _pdy * _s)
                if in_grid(_c):
                    if mind.grid[_c] != LOW:
                        mind.grid[_c] = WALL
                        mind.evid[_c] = 26
                        _newly.add(_c)
                _s += 0.05
            # the gap (the panel's SHUT footprint)
            _s = 0.04
            while _s < 0.38:
                _gx = _d["hx"] + _d["pdx"] * _s
                _gy = _d["hy"] + _d["pdy"] * _s
                _c = cell_of(_gx, _gy)
                if in_grid(_c) and _c not in _newly and mind.grid[_c] != LOW:
                    if _deg >= 50.0:
                        mind.grid[_c] = FREE
                        mind.evid[_c] = -40
                        mind.bump[_c] = 0
                    elif _deg <= 25.0:
                        mind.grid[_c] = WALL  # a shut door IS a wall --
                        mind.evid[_c] = 20    # and remembered-FREE-now-
                        # WALL is exactly what arms the door-suspect
                        # ladder (UNK here silently disarmed RUN 2's
                        # 'map REMEMBERS an open passage' mechanism)
                _s += 0.05
            door_painted[_d["name"]] = _newly

    def _door_is_banned(dr):
        _c = cell_of(dr["x"], dr["y"])
        return any(abs(_c[0] - c0) <= 2 and abs(_c[1] - c1) <= 2
                   for c0, c1 in door_banned)

    def _retire_suspicion(dr):
        # A door he has OPENED must stop being a 'map REMEMBERS an open
        # passage' suspect: was_free is what re-arms that ladder, and
        # his own traffic nudging the panel re-armed it forever (RUN 2
        # spent its whole clock re-conquering its entry door). Curiosity
        # and the lidar-beats-floorplan path still handle it if it truly
        # blocks him again -- this only retires the MEMORY suspicion.
        if was_free is None:
            return
        _dc = cell_of(dr["x"], dr["y"])
        for _di in range(-2, 3):
            for _dj in range(-2, 3):
                _c = (_dc[0] + _di, _dc[1] + _dj)
                if in_grid(_c):
                    was_free[_c] = False

    def _door_slate(c):
        # a deliberate ram/push is not an obstacle encounter: bump marks
        # are TIME-decayed, so his own battering was blockading doorways
        # long after the ring saw daylight. (Run-level: the pushing
        # detector fires outside the door-scout branch -- the nested def
        # crashed the yaw evolution on the doorless maze.)
        for di in range(-2, 3):
            for dj in range(-2, 3):
                cc = (c[0] + di, c[1] + dj)
                if in_grid(cc):
                    mind.bump[cc] = 0

    door_banned = set()        # cells whose door DEFEATED him this run --
    # the full tragedy played, he shook it off and walked away; he does
    # not go back (run-scoped: doors are dynamic, next run he may retry)
    was_free = (mind.grid == FREE).copy() if (race and mind_path) else None
    # the LOADED memory's free cells: any of them that later reads WALL
    # is a shut-door suspect — "he's been through this passage before,
    # but now the way is blocked" (Bill). Found at map level because
    # the route-blocked recognizer was a catch-22: the map re-marks the
    # door WALL, the path through it dies, and no path = no trigger.
    stuck_pts = deque(maxlen=24)   # past stuck spots ("I don't fit" memory)
    grind_n, grind_at = 0, None
    jitter = 0.0
    zoom_arm = 0
    zooming = False
    last_carrot = None
    ring_hist = [deque(maxlen=RING_WIN) for _ in range(24)]
    sonar_hist = deque(maxlen=RING_WIN)
    climb_until = -1.0
    climb_from = None
    climb_arm = 0
    front_hist = deque(maxlen=RING_WIN)
    left_hist = deque(maxlen=RING_WIN)
    right_hist = deque(maxlen=RING_WIN)
    planted_since = None
    next_dbg = 0.0
    t = 0.0
    events = []
    frames, renderer, viewer = [], None, None
    if video:
        renderer = mujoco.Renderer(model, 540, 960)
    elif frame_cb is not None:
        # spectator feed: offscreen tracking camera into a persistent window
        # (Bill: "it kept opening new windows" — the native viewer can't be
        # reused across runs; a rendered feed can)
        renderer = mujoco.Renderer(model, 400, 640)
    if view:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data)
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = model.body("base").id
        viewer.cam.distance, viewer.cam.elevation = 1.0, -40
    pending_shifts = list(shifts or [])
    doubt_cool = -1.0
    wall0 = time.perf_counter()
    n = int((max_s or SIM_MAX_S) / model.opt.timestep)
    result = dict(done=False, scouts=0, bans=0, reason="timeout",
                  wall_hits=0, wall_s=0.0)
    wall_touching = False
    glance_pts = []            # rising-edge contact positions (spectator map)
    sat_knee, sat_n = 0, 0
    sat_by = {}
    # TUNNEL AUDIT (Bill's foot audits, made permanent): body track + extreme
    # foot reach through the exit-tunnel zone land in the result — the 2-4 cm
    # north bias was invisible in aggregates until a trace was pulled by hand.
    foot_gid = [model.geom(f"foot{i}").id for i in range(6)]
    tun = []
    for s in range(n):
        if s % CTRL_EVERY == 0:
            sd = data.sensordata
            sonar_hist.append(2.0 if sd[0] < 0 else float(sd[0]))
            sonar = max(sonar_hist)      # legs blip the sonar path too
            ring = [2.0 if v < 0 else float(v) for v in sd[1:25]]
            q = data.body("base").xquat
            yawr = yaw_of(q)
            bx, by = float(data.body("base").xpos[0]), float(data.body("base").xpos[1])
            if os.environ.get("EDGE_DBG2") and 14.0 < t < 28.0 and int(t*100) % 50 == 0:
                for _ci in range(data.ncon):
                    _c = data.contact[_ci]
                    _g1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, _c.geom1) or str(_c.geom1)
                    _g2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, _c.geom2) or str(_c.geom2)
                    if "floor" in (_g1, _g2):
                        continue
                    print(f"CDBG t={t:.2f} {_g1} <-> {_g2} at ({_c.pos[0]:+.3f},{_c.pos[1]:+.3f},{_c.pos[2]:+.3f})")
            # ---- WALL-GLANCE AUDIT (Bill: "too much allowance in his
            # tactical diameter... glancing off the sides of the walls") —
            # contact between any robot part and a STATIC world geom that
            # isn't the floor plane. Counted as rising edges + total seconds;
            # the panic fitness prices each glance, so evolution re-tightens
            # the drift itself instead of selling margin for clock time.
            # (Caveat: corr_obs bars are world geoms too — a legit climb
            # would count; evolution worlds are the base maze, no bars.)
            _touch = False
            for _ci in range(data.ncon):
                _c = data.contact[_ci]
                _b1 = model.geom_bodyid[_c.geom1]
                _b2 = model.geom_bodyid[_c.geom2]
                if (_b1 == 0) == (_b2 == 0):
                    continue                      # robot-self or world-world
                # ...but "not body 0" is not the same as "the robot". A
                # hinged DOOR PANEL is its own body, so panel-vs-wall
                # contact was scoring as a robot wall-glance: maze3
                # reported wall_s 265.17 on a 265.2 s run (nvd3g touching
                # mvOE1 for 534 contact-seconds). Any tuning that read
                # wall_s in a door world was reading noise. Require the
                # moving side to actually be a ROBOT body.
                _rb = _b1 if _b1 != 0 else _b2
                if _rb not in robot_bodies:
                    continue
                _wg = _c.geom1 if _b1 == 0 else _c.geom2
                if model.geom_type[_wg] != 0:     # not the floor plane
                    _touch = True
                    break
            if _touch:
                result["wall_s"] = round(result["wall_s"] + 0.01, 2)
                if not wall_touching:
                    result["wall_hits"] += 1
                    if len(glance_pts) < 400:
                        glance_pts.append((round(bx, 2), round(by, 2)))
            wall_touching = _touch
            if ((scout is not None or t < measure_until)
                    and s % (CTRL_EVERY * 5) == 0 and len(scan3d_new) < 4000):
                # the 3D his scans actually paint (Bill: "any way to see the
                # 3D view he develops from gimbal scans?") — ray endpoints
                # from the TRUE tilted sensor sites, only while scanning:
                # no omniscience, this is what he touched and nothing more
                for _sid, _adr in _rf3d:
                    _r3 = float(data.sensordata[_adr])
                    if 0.05 < _r3 < 1.9:
                        _sp = (data.site_xpos[_sid]
                               + data.site_xmat[_sid].reshape(3, 3)[:, 2] * _r3)
                        scan3d_new.append((round(float(_sp[0]), 3),
                                           round(float(_sp[1]), 3),
                                           round(float(_sp[2]), 3)))
            here = cell_of(bx, by)
            if not in_grid(here):
                # a wild race mutant once left the grid entirely and a
                # bare grid[here] ate index -19 (crashed an evolution
                # at gen 7) — clamp to the nearest in-grid cell
                here = (max(0, min(GNX - 1, here[0])),
                        max(0, min(GNY - 1, here[1])))
            slope_on = att_on or (abs(drv.body_pitch)
                                  + abs(drv.body_roll) > 0.03)
            _sa = 0.0066
            slope_mp += _sa * (drv.body_pitch - slope_mp)
            slope_mr += _sa * (drv.body_roll - slope_mr)
            slope_ma += _sa * ((abs(drv.body_pitch)
                                + abs(drv.body_roll)) - slope_ma)
            slope_hold = (slope_ma > 0.02
                          and (abs(slope_mp) + abs(slope_mr))
                          / (slope_ma + 1e-6) > 0.5)
            # slope_hold = PERSISTENT one-signed lean (a hill) vs the deck's
            # sign-alternating rock: only a persistent bias poisons the map
            # (the deck's phantoms self-contradict half a cycle later and
            # decay heals). Mapping pauses only on slope_hold; the crouch-
            # trust gates keep the broader slope_on. Computed on COMMANDED
            # pose — finally a signal the servo can't erase.
            # SLOPE-AWARE MAPPING: the signal is the COMMANDED pose —
            # measured tilt collapses once the servo levels him (the
            # loop erases its own evidence; killed five discriminators
            # and disarmed the v1 gate mid-slope). While slope_on, the
            # ring stares into the hill: walk, don't paint, don't
            # crouch — the truth returns on the flat and decay heals.
            on_line = False   # line-following a carve this tick (drive uses it)
            # (NEGATIVE RESULTS, kept for the record: a body-squaring PD
            # split (align to lane axis while velocity flies the curve) cost
            # 9 s and a third more glances; a drift snap-off (kill vx once
            # |yaw_err|<0.25) cost 24 s and TRIPLED glances. The evolved
            # cornering equilibrium is tightly co-adapted — hand-authored
            # choreography loses; give evolution tools + prices instead.)
            mouth_facing = False  # heading roughly along the lane axis — veer
            # yields only then; the oblique return leg keeps corner protection
            near_mouth = False  # in a lane's approach: the LINE is the only
            # centering authority there — ring nudges (corridor centering,
            # corner veer) see the flanking walls asymmetrically and shove
            # him off the lane (the tug-of-war behind the mouth chaos)

            for sh in list(pending_shifts):
                if sh["when"](t, bx, by):
                    sh["apply"](model)
                    pending_shifts.remove(sh)
                    events.append((round(t,1), f"WORLD SHIFTED: {sh['name']} (he doesn't know yet)"))

            # rolling ring window: cleared on hard pivots only (at 0.32 rad/s a
            # 0.3 s window sweeps <7 deg — under half a bearing bin), so gentle
            # path-tracking corrections don't starve the map
            if abs(drv.omega) > 0.32:
                for h in ring_hist:
                    h.clear()
            else:
                for b in range(24):
                    ring_hist[b].append(ring[b])
            ok = [len(h) == RING_WIN for h in ring_hist]
            rmin = [min(h) if h else 2.0 for h in ring_hist]
            rmax = [max(h) if h else 2.0 for h in ring_hist]
            # arcs from PER-BEARING window maxima: a knee blip is transient so
            # the bearing's max erases it, but a persistent 5 cm return is a
            # REAL contact (wall C's corner lived under a blanket self-filter
            # and wedged him for 700 s). Front cone widened to +/-30 deg so
            # corner approaches are seen before the flank drags.
            wmax = [max(h) if len(h) == RING_WIN else None for h in ring_hist]
            def arc(idxs):
                vals = [wmax[i % 24] for i in idxs if wmax[i % 24] is not None]
                return min(vals) if vals else 2.0
            front = arc((22, 23, 0, 1, 2))
            left = arc((18, 19, 20))
            right = arc((4, 5, 6))
            ldiag = arc((20, 21, 22))
            rdiag = arc((2, 3, 4))

            roll = math.atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]**2+q[2]**2))
            pitch = math.asin(max(-1.0, min(1.0, 2*(q[0]*q[2]-q[3]*q[1]))))
            tilted = abs(roll) + abs(pitch) > 0.10
            if abs(drv.v) < 0.004 and abs(drv.omega) < 0.05:
                if planted_since is None:
                    planted_since = t
            else:
                planted_since = None
            planted = planted_since is not None and t - planted_since > 0.4

            if t >= bump_decay_at:
                np.subtract(mind.bump, 1, out=mind.bump)
                np.maximum(mind.bump, 0, out=mind.bump)
                bump_decay_at = t + 10.0
            if dbg and t >= next_dbg:
                ye = wrap(yawr - heading) if heading is not None else 0.0
                print(f"DBG t={t:6.1f} pos=({bx:+.2f},{by:+.2f}) cov={mind.coverage():3d}"
                      f" goal={mind.goal} plen={len(mind.path):2d}"
                      f" yaw={yawr:+.2f} err={ye:+.2f} v={drv.v:+.3f} om={drv.omega:+.2f}"
                      f" front={front:.2f} mode={'SCAN' if scout else ('GOTO-SCAN' if scan_next else 'NAV')}")
                next_dbg = t + 5.0
                if int(t) % 60 < 5:
                    chars = {UNK: '.', FREE: ' ', WALL: '#', LOW: '~'}
                    for j in range(GNY-1, -1, -1):
                        print("MAP " + "".join('G' if (i, j) == here else chars[mind.grid[(i, j)]]
                                               for i in range(GNX)))

            # ---- SCANNING, two speeds ----
            # CROUCH PEEK first (3 s): drop low and compare what the ring SEES
            # against what the map EXPECTS — a bearing that sees 0.3 m past
            # its believed wall is a tunnel. Cheap, and it is belief revision
            # in its purest form. Ambiguous -> escalate to the full gimbal.
            if scout is not None and scout["mode"] == "door":
                # THE SHUT-DOOR LADDER (Bill: "he needs to recognize it's a
                # door that should open... tries to kick it open first, then
                # uses his body to lean on it and push with his front legs,
                # then maybe tries to pop it open by aggressively pushing")
                drv.vx = 0.0; drv.omega = 0.0; drv.height = 0.0
                drv.lift = cr["lift"]
                stuck_hist.clear()
                st = scout["stage"]
                if st == "thru":
                    # walk to an OPEN door and surge through the gap
                    _dr4 = scout["door"]
                    _hd4 = _dr4["hd"]
                    if ((_dr4["x"] - bx) * -math.sin(_hd4)
                            + (_dr4["y"] - by) * math.cos(_hd4)) < 0:
                        _hd4 = wrap(_hd4 + math.pi)
                    _dd4 = math.hypot(_dr4["x"] - bx, _dr4["y"] - by)
                    if front < 0.16 and _dd4 > 0.34:
                        # THAT IS A WALL, NOT THE DOOR. The geometry walk
                        # was open-loop: bearing + a timeout, never once
                        # consulting the sonar sitting right there.
                        events.append((round(t,1), f"walking at {_dr4['name']} by geometry -> BLOCKED {_dd4:.2f} m out: that is a WALL, not the door"))
                        door_tried_at[_dr4["name"]] = t
                        scout = None
                        door_cool = t + 8.0
                        replan_at = 0.0
                        mind.path = []
                    elif _dd4 < 0.30 or t - scout["t0"] > 12.0:
                        surge_until = t + 5.5
                        surge_hd = _hd4
                        door_done_names.add(_dr4["name"])
                        _door_slate(cell_of(_dr4["x"], _dr4["y"]))
                        _door_slate(cell_of(bx, by))
                        scout = None
                        replan_at = t + 5.5
                        mind.path = []
                    else:
                        _hda4 = math.atan2(-(_dr4["x"] - bx), (_dr4["y"] - by))
                        _fe4 = wrap(_hda4 - yawr)
                        drv.omega = float(np.clip(2.0 * _fe4, -0.5, 0.5))
                        drv.v = cr["speed"] * (0.035 if abs(_fe4) < 0.6 else 0.0)
                if st == "approach":
                    # walk to the STATION: 0.26 m back from the door
                    # centre along its normal. Centred matters -- the
                    # first field run stopped 'near' the door but offset
                    # in front of the wall segment, slammed the wall
                    # six times, and a tier-1 door 'defeated' him.
                    _dr2 = scout["door"]
                    _hdv2 = scout["hd"]
                    _sx = _dr2["x"] + math.sin(_hdv2) * 0.26
                    _sy = _dr2["y"] - math.cos(_hdv2) * 0.26
                    _dd2 = math.hypot(_sx - bx, _sy - by)
                    if front < 0.16 and _dd2 > 0.30:
                        # blocked well short of the station: abandon, do
                        # NOT hand the shut-door ladder a wall to kick
                        events.append((round(t,1), f"walking at {_dr2['name']} by geometry -> BLOCKED {_dd2:.2f} m out: that is a WALL, not the door"))
                        door_tried_at[_dr2["name"]] = t
                        scout = None
                        door_cool = t + 8.0
                        replan_at = 0.0
                        mind.path = []
                    else:
                        _hda = math.atan2(-(_sx - bx), (_sy - by))
                        _fe3 = wrap(_hda - yawr)
                        drv.omega = float(np.clip(2.0 * _fe3, -0.5, 0.5))
                        drv.v = cr["speed"] * (0.035 if abs(_fe3) < 0.6 else 0.0)
                        if _dd2 < 0.09 or t - scout["t0"] > 10.0:
                            drv.v = 0.0
                            scout["stage"], scout["t0"] = "face", t
                if st == "face":
                    drv.v = 0.0
                    _fe = wrap(scout["hd"] - yawr)
                    drv.omega = float(np.clip(2.0 * _fe, -0.42, 0.42))
                    if abs(_fe) < 0.22:
                        drv.omega = 0.0
                        scout["stage"], scout["t0"] = "kick", t
                        scout["x0"], scout["y0"] = bx, by
                        events.append((round(t,1), "SHUT DOOR? the map remembers this open -> KICK first"))
                if st == "kick":
                    drv.v = 0.0
                    if not scout.get("kicked"):
                        scout["kicked"] = True
                        drv.kick_leg = 1              # front-left strikes
                    _kt2 = t - scout["t0"]
                    drv.body_dy = 0.022 if 0.15 < _kt2 < 0.75 else 0.0
                    # gym lesson: the last centimeter of strike reach lives
                    # in a body surge, not the leg
                    if _kt2 > 1.1:
                        if front > 0.45:
                            events.append((round(t,1), "the KICK popped it -> through"))
                            if scout.get("door") is not None:
                                door_done.add(cell_of(scout["door"]["x"], scout["door"]["y"]))
                                door_done_names.add(scout["door"]["name"])
                                _wd = abs(math.degrees(float(data.qpos[scout["door"]["adr"]])))
                                model.dof_frictionloss[scout["door"]["dof"]] = max(
                                    1.5 if _wd >= 72.0 else 0.5,
                                    float(model.dof_frictionloss[scout["door"]["dof"]]))
                            _door_slate(scout["cell"])
                            scout = None; door_cool = t + 6.0
                            replan_at = 0.0; mind.path = []
                        else:
                            _nd = None
                            _cands = [_d for _d in door_reg
                                      if not _door_is_banned(_d)]
                            if _cands:
                                # a door that DEFEATED him is banned --
                                # the nearest-door pick ignored bans and
                                # re-laddered the locked door forever
                                # (RUN 3: two full tragedies, same door)
                                _nd = min(_cands,
                                          key=lambda d: (d["x"]-bx)**2 + (d["y"]-by)**2)
                                if (_nd["x"]-bx)**2 + (_nd["y"]-by)**2 > 0.55**2:
                                    _nd = None
                                elif _nd["name"] in door_done_names:
                                    # already conquered: no mounts, no
                                    # tragedy -- a SWEEP-ONLY reopen (his
                                    # own body may have brushed it back)
                                    events.append((round(t,1), f"{_nd['name']} is already MINE -> sweep it back open"))
                                    scout["door"] = _nd
                                    scout["stage"], scout["t0"] = "resweep", t
                                    _nd = "handled"
                                elif abs(math.degrees(float(data.qpos[_nd["adr"]]))) > 40.0:
                                    # the door is standing OPEN = a
                                    # PASSAGE. Walk THROUGH it (he was
                                    # 'replanning away' from doors his
                                    # own wall-push had swung open, then
                                    # wandering 400 s)
                                    events.append((round(t,1), f"{_nd['name']} is standing OPEN -> walking THROUGH it"))
                                    _hdo = _nd["hd"]
                                    if ((_nd["x"] - bx) * -math.sin(_hdo)
                                            + (_nd["y"] - by) * math.cos(_hdo)) < 0:
                                        _hdo = wrap(_hdo + math.pi)
                                    surge_until = t + 4.5
                                    surge_hd = _hdo
                                    door_done_names.add(_nd["name"])
                                    _door_slate(scout["cell"])
                                    _door_slate(cell_of(_nd["x"], _nd["y"]))
                                    door_cool = t + 6.0
                                    scout = None
                                    replan_at = t + 4.5
                                    mind.path = []
                                    _nd = None
                            if _nd == "handled":
                                pass    # resweep armed -- the trailing
                                # legacy branch was OVERWRITING the stage
                                # with 'lean' in the same tick, so every
                                # 'already MINE' trial ran the pre-breach
                                # pop ladder in an infinite loop
                            elif _nd is not None:
                                # a REGISTERED door: the gym-certified
                                # breach (BreachFSM), not the old lean.
                                # Square to the door's NORMAL, resolved to
                                # his side -- not the approach diagonal.
                                _hdv = _nd["hd"]
                                if ((_nd["x"] - bx) * -math.sin(_hdv)
                                        + (_nd["y"] - by) * math.cos(_hdv)) < 0:
                                    _hdv = wrap(_hdv + math.pi)
                                scout["hd"] = _hdv
                                scout["door"] = _nd
                                scout["stage"], scout["t0"] = "closein", t
                                events.append((round(t,1), f"still shut -> THE BREACH on {_nd['name']} (closing in)"))
                            elif scout is not None:
                                scout["stage"], scout["t0"] = "lean", t
                                events.append((round(t,1), "still shut -> REAR UP: fronts on the door, back four driving"))
                elif st == "resweep":
                    # shoulder-sweep a conquered door back open: lean
                    # into the panel and keep walking; never mount, never
                    # ban -- worst case slate and route around
                    _dr3 = scout["door"]
                    _q3 = float(data.qpos[_dr3["adr"]])
                    _cq3, _sq3 = math.cos(_q3), math.sin(_q3)
                    _pdx3 = _dr3["pdx"] * _cq3 - _dr3["pdy"] * _sq3
                    _pdy3 = _dr3["pdx"] * _sq3 + _dr3["pdy"] * _cq3
                    _mx3 = _dr3["hx"] + 0.19 * _pdx3
                    _my3 = _dr3["hy"] + 0.19 * _pdy3
                    _le3 = (_mx3 - bx) * math.cos(yawr) + (_my3 - by) * math.sin(yawr)
                    drv.v = 0.036
                    drv.vx = 0.028 if _le3 > 0 else -0.028
                    drv.lift = 0.016
                    if abs(math.degrees(_q3)) > 62.0:
                        drv.vx = 0.0
                        events.append((round(t,1), f"{_dr3['name']} swept to {abs(math.degrees(_q3)):.0f} deg -> onward"))
                        _retire_suspicion(_dr3)
                        _door_slate(scout["cell"])
                        _door_slate(cell_of(bx, by))
                        door_cool = t + 6.0
                        scout = None
                        replan_at = 0.0
                        mind.path = []
                    elif t - scout["t0"] > 6.0:
                        # SWEEP HONESTY: 39 deg is not 'onward' -- a
                        # sweep that cannot swing it wide hands the
                        # door to the breach ladder (it has the whole
                        # arsenal; the lie cost RUN 2 four sweep cycles
                        # on the same half-shut door)
                        drv.vx = 0.0
                        events.append((round(t,1), f"{_dr3['name']} stuck at {abs(math.degrees(_q3)):.0f} deg after the sweep -> the LADDER"))
                        _hdv9 = _dr3["hd"]
                        if ((_dr3["x"] - bx) * -math.sin(_hdv9)
                                + (_dr3["y"] - by) * math.cos(_hdv9)) < 0:
                            _hdv9 = wrap(_hdv9 + math.pi)
                        scout["hd"] = _hdv9
                        scout["stage"], scout["t0"] = "closein", t
                elif st == "closein":
                    # jam the nose: the mount wants gym-spawn standoff
                    # ("he can only get so close to the wall")
                    drv.v = cr["speed"] * 0.030
                    _fe2 = wrap(scout["hd"] - yawr)
                    drv.omega = float(np.clip(1.5 * _fe2, -0.3, 0.3))
                    if front < 0.26 or t - scout["t0"] > 3.0:
                        drv.v = 0.0
                        drv.omega = 0.0
                        scout["fsm"] = DG.BreachFSM(t, hint=scout["door"].get("gainer"))
                        events.append((round(t,1), "jammed -> BreachFSM engaged: the gym choreography, in the field"))
                        scout["stage"] = "breach"
                elif st == "breach":
                    _dreg = scout["door"]
                    _hd0 = scout["hd"]

                    def _shove_pose():
                        # station squared onto the panel's CURRENT face:
                        # midpoint = hinge + rotated half-length; stand
                        # 0.27 m off it along the panel normal on HIS
                        # side, facing the panel
                        _q = float(data.qpos[_dreg["adr"]])
                        _cq, _sq = math.cos(_q), math.sin(_q)
                        _pdx = _dreg["pdx"] * _cq - _dreg["pdy"] * _sq
                        _pdy = _dreg["pdx"] * _sq + _dreg["pdy"] * _cq
                        _mx2 = _dreg["hx"] + 0.19 * _pdx
                        _my2 = _dreg["hy"] + 0.19 * _pdy
                        _nx, _ny = _pdy, -_pdx          # a panel normal
                        if (bx - _mx2) * _nx + (by - _my2) * _ny < 0:
                            _nx, _ny = -_nx, -_ny       # his side
                        _tx = _mx2 + 0.27 * _nx
                        _ty = _my2 + 0.27 * _ny
                        _fwx, _fwy = -math.sin(yawr), math.cos(yawr)
                        _fe = (_tx - bx) * _fwx + (_ty - by) * _fwy
                        _le = (_tx - bx) * math.cos(yawr) + (_ty - by) * math.sin(yawr)
                        _hdp = math.atan2(_nx, -_ny)    # face the panel
                        return (_fe, _le, wrap(_hdp - yawr))

                    _st2 = scout["fsm"].tick(
                        t, drv,
                        lambda: abs(math.degrees(float(data.qpos[_dreg["adr"]]))),
                        lambda rev: wrap(_hd0 + (math.pi if rev else 0.0) - yawr),
                        events.append, shove_pose=_shove_pose)
                    if _st2 != 'working':
                        _dreg["gainer"] = scout["fsm"].gainer   # remember
                        # what worked (or what was working) at THIS door
                        drv.press = 0
                        drv.leg_h = [0.0] * 6
                        drv.body_dy = 0.0
                        drv.body_pitch = 0.0
                        drv.body_yaw = 0.0
                        drv.height = 0.0
                        drv.ram_full = False
                        if _st2 == 'through':
                            events.append((round(t,1), f"BREACHED {_dreg['name']} -> through"))
                            _door_slate(scout["cell"])
                            _door_slate(cell_of(bx, by))
                            surge_until = t + 4.0
                            surge_hd = scout["hd"]
                            door_done.add(cell_of(_dreg["x"], _dreg["y"]))
                            door_done_names.add(_dreg["name"])
                            _retire_suspicion(_dreg)
                            # PIN IT: a door shoved open stays where it
                            # swung (raised hinge friction) -- his own
                            # body brushing through was knocking 60-deg
                            # doors back to 40 and he then performed the
                            # full tragedy on his own conquered door.
                            # GRADED: hard-pin only a truly clear door;
                            # a 55-70 deg panel pinned hard once jammed
                            # the west corridor for 369 s (RUN 3) -- pin
                            # those soft, so a deliberate sweep still
                            # moves them
                            _wide = abs(math.degrees(float(data.qpos[_dreg["adr"]]))) >= 72.0
                            model.dof_frictionloss[_dreg["dof"]] = max(
                                1.5 if _wide else 0.5,
                                float(model.dof_frictionloss[_dreg["dof"]]))
                            replan_at = t + 4.0   # follow through FIRST
                            door_cool = t + 6.0
                        else:
                            events.append((round(t,1), f"DEFEATED by {_dreg['name']} -> BANNED, back to searching"))
                            door_banned.add(scout["cell"])
                            door_banned.add(cell_of(_dreg["x"], _dreg["y"]))
                            # SEARCH THIS WALL (the scenario's whole
                            # second act): a door that will not open is
                            # a reason to doubt the WALL it sits in --
                            # bypasses live next to doors. Demote that
                            # wall's cells to unknown so his frontier
                            # hunt re-examines the span (the vault's
                            # crouch tunnel is 5 cells down from the
                            # locked door, and he never looked).
                            _dc6 = cell_of(_dreg["x"], _dreg["y"])
                            _vert6 = abs(_dreg["pdy"]) > abs(_dreg["pdx"])
                            _n6 = 0
                            # ONLY when he is actually BLOCKED. Demoting
                            # a wall on every defeat spawned phantom
                            # frontiers and cost RUN 1 150 s and RUN 2
                            # its finish (measured). If a route to the
                            # exit still exists, take it -- doubt the
                            # wall only when the door was the last way.
                            # RACE MODE ONLY, and only when truly
                            # blocked. In an EXPLORE run he is still
                            # mapping -- demoting a wall there is pure
                            # noise that spawns phantom frontiers, and
                            # it cost RUN 1 150 s (measured 265 -> 416).
                            # In a memory run the map is complete, so
                            # doubting the wall the door sits in is a
                            # real inference.
                            if (not race) or mind.plan_exit(here) is not None:
                                _dc6 = None
                            for _s6 in (range(-6, 7) if _dc6 is not None else ()):
                                _c6 = ((_dc6[0], _dc6[1] + _s6) if _vert6
                                       else (_dc6[0] + _s6, _dc6[1]))
                                if not in_grid(_c6) or _c6 == _dc6:
                                    continue
                                if mind.grid[_c6] == WALL:
                                    mind.grid[_c6] = UNK
                                    mind.evid[_c6] = 0
                                    _n6 += 1
                            if _n6:
                                events.append((round(t,1), f"...but a wall with one door may have another way through -> re-examining {_n6} cells of it"))
                            scan_cool = t
                            scanned_pts.clear()
                            door_cool = t + 8.0
                        scout = None
                        replan_at = 0.0
                        mind.path = []
                elif st == "lean":
                    # the SIMPLE lean — press + modest pitch + drive. The
                    # gym-drilled rear-surge sequence (door_gym) looks right
                    # in the gym but transplanted badly (whiffed air from
                    # arrival geometry, 126 s -> 868 s): field-porting the
                    # theatrical rear is a NAMED OPEN ITEM; the door still
                    # opens the boring way.
                    drv.press = 1
                    drv.body_pitch = 0.15
                    drv.v = cr["speed"] * 0.035
                    if t - scout["t0"] > 3.5:
                        drv.press = 0; drv.body_pitch = 0.0; drv.body_dy = 0.0
                        _mv = math.hypot(bx - scout["x0"], by - scout["y0"])
                        if front > 0.50 or _mv > 0.30:
                            events.append((round(t,1), f"the PRESS opened it ({_mv:.2f} m) -> through"))
                            _door_slate(scout["cell"])
                            scout = None; door_cool = t + 6.0
                            replan_at = 0.0; mind.path = []
                        else:
                            scout["stage"], scout["t0"] = "pop", t
                            events.append((round(t,1), "still shut -> POP it: full brace, ram pulses"))
                elif st == "pop":
                    _ph = (t - scout["t0"]) % 1.4
                    if _ph < 0.55:
                        drv.press = 0
                        drv.v = -0.5 * cr["speed"] * 0.035
                    else:
                        drv.press = 2                 # fronts + middles braced
                        drv.v = cr["speed"] * 0.035
                        # follow the ARC: a half-open door angles across the
                        # gap and dead-straight lunges slide along its face.
                        # Alternate a lateral bias; the productive side is
                        # the one that keeps gaining reach.
                        drv.vx = (0.012 if scout.get("rounds", 0) % 2 else -0.012)
                    scout["mvmax"] = max(scout.get("mvmax", 0.0),
                                         math.hypot(bx - scout["x0"],
                                                    by - scout["y0"]))
                    # furthest REACH, not endpoint: the back-off phase
                    # returns him to the start, so endpoint displacement
                    # was blind to the door creeping open between lunges
                    if t - scout["t0"] > 4.3:
                        _mv = scout["mvmax"]
                        if front > 0.50 or _mv > 0.30:
                            drv.press = 0
                            events.append((round(t,1), f"POPPED it ({_mv:.2f} m) -> through"))
                            _door_slate(scout["cell"])
                            door_cool = t + 6.0
                            scout = None
                            replan_at = 0.0; mind.path = []
                        elif (_mv - scout.get("mv0", 0.0) > 0.03
                                and t - scout.get("t00", scout["t0"]) < 14.0):
                            # it's GIVING — don't quit a door that moves
                            # (the honest 0.30 bar made him abandon a door
                            # he'd shoved 0.10 m open). Keep popping.
                            scout.setdefault("t00", scout["t0"])
                            scout["mv0"] = _mv
                            scout["t0"] = t
                            scout["side"] = (0.012 if scout.get("rounds", 0) % 2
                                             else -0.012)   # the side that gained
                            scout["rounds"] = scout.get("rounds", 0) + 1
                            events.append((round(t,1), f"it's GIVING ({_mv:.2f} m) -> keep popping"))
                        elif _mv > 0.08:
                            drv.press = 0
                            events.append((round(t,1), f"half open ({_mv:.2f} m) -> SLIP the gap"))
                            _door_slate(scout["cell"])
                            scout["stage"], scout["t0"] = "slip", t
                        else:
                            drv.press = 0
                            drv.vx = 0.0
                            events.append((round(t,1), "won't budge -> not a door after all"))
                            door_cool = t + 30.0
                            scout = None
                            replan_at = 0.0; mind.path = []
                elif st == "slip":
                    # he's won the fight — now walk through the prize: the
                    # gap is BESIDE the half-open panel, on the side the pop
                    # rounds learned was productive. Thread it diagonally.
                    drv.press = 0
                    drv.v = cr["speed"] * 0.030
                    drv.vx = scout.get("side", -0.012) * 1.6
                    if t - scout["t0"] > 2.8:
                        drv.vx = 0.0
                        _mv2 = (-(bx - scout["x0"]) * math.sin(scout["hd"])
                                + (by - scout["y0"]) * math.cos(scout["hd"]))
                        if _mv2 > 0.28:
                            events.append((round(t,1), f"SLIPPED through ({_mv2:.2f} m)"))
                            _door_slate(scout["cell"])
                            _door_slate(cell_of(bx, by))
                            # squeezing through grazes the panel — those
                            # contacts are credentials, not obstacles; fresh
                            # bump marks laid DURING the slip blockaded
                            # plan_exit and he wandered back out the door
                            door_cool = t + 6.0
                        else:
                            events.append((round(t,1), "gap too tight -> back off, let the map see"))
                            door_cool = t + 10.0
                            back_until = t + 1.2
                        scout = None
                        replan_at = t + 0.5; mind.path = []
            elif scout is not None and scout["mode"] == "push":
                drv.lift, drv.height = cr["lift"], 0.0
                drv.vx = 0.0
                drv.omega = 0.0                  # hold this heading, drive
                drv.v = cr["speed"] * 0.035      # firm, full cruise
                stuck_hist.clear()               # a deliberate ram isn't stuck
                if t - scout["t0"] > (2.5 if scout.get("through") else 3.5):
                    moved = math.hypot(bx - scout["x0"], by - scout["y0"])
                    if scout.get("through"):
                        scout = None             # we're through — replan fresh
                        scan_cool = t + 4.0
                        replan_at = 0.0
                        mind.path = []
                    elif front > 0.50 or moved > 0.30:
                        events.append((round(t,1), f"DOOR: it YIELDS ({moved:.2f} m) -> pushing through"))
                        scout["through"] = True
                        scout["t0"] = t
                        scout["x0"], scout["y0"] = bx, by
                    else:
                        _pd = None
                        if door_reg:
                            _pd = min(door_reg,
                                      key=lambda d: (d["x"]-bx)**2 + (d["y"]-by)**2)
                            if ((_pd["x"]-bx)**2 + (_pd["y"]-by)**2 > 0.50**2
                                    or abs(math.degrees(float(data.qpos[_pd["adr"]]))) < 25.0):
                                _pd = None
                        if _pd is not None:
                            # the push verdict measures HIS displacement,
                            # not the door's -- the shove had swung a
                            # light door wide while reading 'solid wall'
                            events.append((round(t,1), f"the push swung {_pd['name']} OPEN -> through"))
                            _retire_suspicion(_pd)
                            _hdo2 = _pd["hd"]
                            if ((_pd["x"] - bx) * -math.sin(_hdo2)
                                    + (_pd["y"] - by) * math.cos(_hdo2)) < 0:
                                _hdo2 = wrap(_hdo2 + math.pi)
                            surge_until = t + 4.5
                            surge_hd = _hdo2
                            door_done_names.add(_pd["name"])
                            _door_slate(cell_of(_pd["x"], _pd["y"]))
                            _door_slate(cell_of(bx, by))
                            scout = None
                            scan_cool = t + 4.0
                            replan_at = t + 4.5
                            mind.path = []
                        else:
                            events.append((round(t,1), "solid wall -> next suspect"))
                            scout = None
                            scan_cool = t + 8.0
                            replan_at = 0.0
                            mind.path = []
            elif scout is not None and scout["mode"] == "peek":
                drv.v = drv.vx = 0.0; drv.omega = 0.0
                drv.lift, drv.height = 0.0, DUCK["height"]
                drv.body_pitch = drv.body_roll = 0.0
                if t - scout["t0"] > 0.6 and int((t - scout["t0"]) * 100) % 25 == 0:
                    # ring snapshots through the crouched window: the settle
                    # wobble jitters ray endpoints just enough that folding
                    # every snapshot into the edge stash converges the bracket
                    # in ONE squat (Bill: "he squats twice" — the second squat
                    # only ever re-collected this same diversity)
                    scout.setdefault("buf", []).append((yawr, list(wmax)))
                if t - scout["t0"] > 1.8:
                    mx = bx - math.sin(yawr)*MAST_FWD; my = by + math.cos(yawr)*MAST_FWD
                    best_b, best_ex, best_exp = None, 0.0, 0.3
                    for b in range(24):
                        if wmax[b] is None:
                            continue
                        az = math.radians(15*b)
                        # a tunnel pierces a SOLID WALL SPAN: the believed wall
                        # must hold on three neighboring ray-lines, or this is
                        # a doorway-EDGE artifact (the bearing's center clips a
                        # wall corner while the physical ray slips through the
                        # open gap beside it — the chamber carve/revoke loop)
                        exps = [mind.expected_range(mx, my,
                                                    math.sin(az + da - yawr),
                                                    math.cos(az + da - yawr))
                                for da in (-0.10, 0.0, 0.10)]
                        exp = exps[1]
                        if max(exps) < 0.75 and wmax[b] > exp + 0.35 \
                                and wmax[b] - exp > best_ex:
                            best_ex, best_b, best_exp = wmax[b] - exp, b, exp
                    if best_b is not None:
                        # carve along the MIDDLE of the see-through span, not
                        # the single best ray: a ray-following lane lands
                        # off the tunnel centerline and the alignment gate
                        # then faithfully centers him on the wrong line
                        # (Bill: left middle leg dragging the exit wall)
                        span, wsum = 0.0, 0.0
                        for db in (-1, 0, 1):
                            bb = (best_b + db) % 24
                            if wmax[bb] is None:
                                continue
                            azb = math.radians(15*bb)
                            expb = mind.expected_range(
                                mx, my, math.sin(azb - yawr), math.cos(azb - yawr))
                            gain = wmax[bb] - expb
                            if expb < 0.75 and gain > 0.20:
                                span += db * gain
                                wsum += gain
                        az = math.radians(15*best_b + 15*(span/wsum if wsum else 0.0))
                        scout = dict(mode="confirm", t0=t, az=az, bb=best_b,
                                     ex=best_ex, exp=best_exp, src="peek")
                        # stash edge-bracket samples NOW: crouched + planted =
                        # the ring is under the lintel and the pose is exact —
                        # these flank hits seed the carve's sub-cell centerline
                        dxc, dyc = math.sin(az - yawr), math.cos(az - yawr)
                        # anchor the stash EXACTLY where the carve will anchor
                        # (a mismatched origin put every true flank hit outside
                        # the along-gate: bracket 0/0 while staring at faces)
                        sdc = max(0.1, best_exp - 0.15) + 0.15
                        Pc = (mx + dxc*sdc, my + dyc*sdc)
                        axy = ((math.copysign(1.0, dxc), 0.0)
                               if abs(dxc) > abs(dyc)
                               else (0.0, math.copysign(1.0, dyc)))
                        edge = [1e9, -1e9, -1e9, 1e9, 0, 0, 0]
                        for yb, wm in scout.get("buf", []) + [(yawr, list(wmax))]:
                            for b2 in range(24):
                                if wm[b2] is None:
                                    continue
                                az2 = math.radians(15*b2)
                                Mind.edge_accum(edge, Pc, axy, mx, my,
                                                math.sin(az2 - yb),
                                                math.cos(az2 - yb), wm[b2])
                        if os.environ.get("EDGE_DBG"):
                            print(f"EDBG stash t={t:.1f} mast=({mx:.3f},{my:.3f}) yaw={yawr:.2f} "
                                  f"az={math.degrees(az):.0f} P=({Pc[0]:.2f},{Pc[1]:.2f}) axy={axy}")
                            for b2 in range(24):
                                r2 = wmax[b2]
                                if r2 is None:
                                    continue
                                d2x, d2y = (math.sin(math.radians(15*b2) - yawr),
                                            math.cos(math.radians(15*b2) - yawr))
                                den2 = d2x*axy[0] + d2y*axy[1]
                                dw2 = (-((mx-Pc[0])*axy[0] + (my-Pc[1])*axy[1]) / den2
                                       if abs(den2) > 1e-6 else 9.9)
                                print(f"  b={b2:2d} r={r2:5.2f} den={den2:+.2f} dw={dw2:+5.2f} "
                                      f"end=({mx+d2x*r2:.2f},{my+d2y*r2:.2f})")
                            print(f"  -> edge={['%.2f' % v for v in edge[:4]]} n={edge[4:]}")
                        scout["edge"] = edge
                        events.append((round(t,1), f"CROUCH PEEK: seeing {best_ex:.2f} m PAST the wall I believed at bearing {15*best_b} deg -> STAND-UP CHECK"))
                    else:
                        scout["mode"] = "gimbal"
                        scout["t0"] = t
                        events.append((round(t,1), "peek matches the map -> full gimbal"))
            # ---- VERDICT CONFIRMATION (the scout-mode two-phase lesson,
            # restored): a crouched ray seeing past a believed wall proves an
            # OPENING — not a LOW one. A tall doorway the map mis-believed
            # (edge/corner cells read as solid) fires the exact same peek
            # signature — that is how crouch lanes got carved through the D
            # doorway (crouch time crept 43 -> 79 s/run across the last five
            # base logs). So: RISE ~3 cm (the ring clears its stance sag over
            # the 0.115 m lintel) and look again. Still blocked = low tunnel,
            # carve. Sees through raised = doorway: erode the phantom wall
            # belief and let him route it standing.
            elif scout is not None and scout["mode"] == "confirm":
                drv.v = drv.vx = 0.0; drv.omega = 0.0
                drv.lift, drv.height = 0.0, 0.030
                drv.body_pitch = drv.body_roll = 0.0
                if t - scout["t0"] > 1.0:
                    _door_scout = None
                    az = scout["az"]
                    mx = bx - math.sin(yawr)*MAST_FWD; my = by + math.cos(yawr)*MAST_FWD
                    dxg, dyg = math.sin(az - yawr), math.cos(az - yawr)
                    exp = mind.expected_range(mx, my, dxg, dyg)
                    # raised look: MAX over the 3-bearing span — one corner-
                    # grazing bin (or a body micro-shift while rising) must
                    # not fake "blocked"; a tunnel blocks the WHOLE span
                    bbs = scout["bb"]
                    seen = max((wmax[(bbs+k) % 24] for k in (-1, 0, 1)
                                if wmax[(bbs+k) % 24] is not None), default=2.0)
                    blocked = seen < exp + 0.15
                    if scout["exp"] is not None:
                        # differential test: a REAL low tunnel LOSES the
                        # crouched see-through when raised; a doorway with a
                        # back wall at map-expected depth keeps it (the D-area
                        # false confirm: raised ray reached the wall BEHIND
                        # the gap at ~exp and read as "blocked")
                        blocked = blocked and (seen < scout["exp"] + scout["ex"] - 0.30)
                    if scout["src"] == "peek":
                        scanned_pts.append((bx, by))
                    if blocked and slope_on:
                        events.append((round(t,1), "verdict DEFERRED: on a slope, ring geometry untrusted"))
                    elif blocked:
                        if scout["exp"] is None:
                            gi_new = mind.carve_low(mx, my, yawr, az,
                                                    edge=scout.get("edge"))
                        else:
                            gi_new = mind.carve_low(mx, my, yawr, az,
                                                    max_d=min(scout["ex"] + 0.4, 1.0),
                                                    start_d=max(0.1, scout["exp"] - 0.15),
                                                    edge=scout.get("edge"))
                        events.append((round(t,1), f"CONFIRMED: blocked raised, open crouched -> LOW lane carved at bearing {math.degrees(az):.0f} deg"))
                        # ONE SQUAT DOCTRINE (Bill: "he squats twice"): the
                        # crouch peek WAS the measuring ritual — its stash
                        # seeded the bracket from this very pose, and a second
                        # squat here (tried) or an approach micro-peek later
                        # re-collects the same rays. Consume the mouth's
                        # measuring budget NOW; the moving crouched samples on
                        # the way in are what actually converge the bracket.
                        # Gimbal-sourced carves (no stash) keep their one
                        # approach peek.
                        if scout.get("edge") is not None:
                            g_new = mind.carve_geo[gi_new]
                            k_new = (round(g_new[0], 2), round(g_new[1], 2))
                            mouth_measured[k_new] = mouth_measured.get(k_new, 0) + 1
                    else:
                        # DOORWAY: measured open at two heights. Plant durable
                        # FREE with deep negative evidence — eroding to UNK
                        # fed a scan carousel (UNK spawns frontiers, corner
                        # hits re-wall the mixed cells, the map dries up, scan
                        # again: 10 scouts/run). Negative evidence means ~10
                        # fresh endpoint hits reassert a REAL wall (a shifted
                        # world stays revisable) while stray corner grazes
                        # can't re-seal a verified opening.
                        n_er = 0
                        for da in (-0.10, 0.0, 0.10):
                            dxe, dye = math.sin(az + da - yawr), math.cos(az + da - yawr)
                            d = 0.10
                            while d < exp + 0.35:
                                ce = cell_of(mx + dxe*d, my + dye*d)
                                if in_grid(ce) and mind.grid[ce] != LOW:
                                    mind.evid[ce] = min(mind.evid[ce], -40)
                                    if mind.grid[ce] in (WALL, UNK):
                                        mind.grid[ce] = FREE
                                        n_er += 1
                                d += 0.075
                        events.append((round(t,1), f"CONFIRM FAILED: still see through STANDING -> a doorway, not a tunnel ({n_er} phantom cells held open, no carve)"))
                        # FIRST CONTACT (Bill: "he needs to recognize it's
                        # a door"): a registered door sitting on the
                        # anomaly ray means this is not a mapping puzzle,
                        # it's a SHUT DOOR seen by its hinge-slit. Arm the
                        # ladder -- no was_free memory required. (RUN 1 of
                        # the vault stalled 560 s on exactly this: the
                        # verdict said doorway, nothing acted on it.)
                        if door_reg and t > door_cool:
                            _best = None
                            _thru_dr = None
                            for _dr in door_reg:
                                _dc = cell_of(_dr["x"], _dr["y"])
                                if any(abs(_dc[0] - _c0) <= 2 and abs(_dc[1] - _c1) <= 2
                                       for _c0, _c1 in door_banned):
                                    continue
                                if abs(math.degrees(float(data.qpos[_dr["adr"]]))) > 40.0:
                                    # OPEN door on the anomaly ray: the
                                    # lidar's definitive opening beats
                                    # the floor plan (Bill: 'his lidar
                                    # scans show an opening definitively,
                                    # but his mind floor plan has blocked
                                    # it') -- the swung panel re-paints
                                    # WALL over every verdict erasure, so
                                    # don't fight the map: WALK THROUGH.
                                    _px6 = _dr["x"] - mx
                                    _py6 = _dr["y"] - my
                                    _pj6 = _px6 * dxg + _py6 * dyg
                                    _pp6 = abs(-_px6 * dyg + _py6 * dxg)
                                    if 0.0 < _pj6 < 0.85 and _pp6 < 0.30:
                                        _thru_dr = _dr
                                    continue
                                if _dr["name"] in door_done_names:
                                    continue
                                _px, _py = _dr["x"] - mx, _dr["y"] - my
                                _proj = _px * dxg + _py * dyg
                                _perp = abs(-_px * dyg + _py * dxg)
                                if 0.0 < _proj < 0.85 and _perp < 0.24:
                                    if _best is None or _proj < _best[0]:
                                        _best = (_proj, _dr)
                            if _best is not None:
                                _dr = _best[1]
                                _hdv = _dr["hd"]
                                if ((_dr["x"] - bx) * -math.sin(_hdv)
                                        + (_dr["y"] - by) * math.cos(_hdv)) < 0:
                                    _hdv = wrap(_hdv + math.pi)
                                _door_scout = dict(mode="door", stage="approach",
                                                   t0=t, x0=bx, y0=by,
                                                   cell=cell_of(_dr["x"], _dr["y"]),
                                                   hd=_hdv, door=_dr)
                                door_cool = t + 4.0
                                events.append((round(t,1), f"...that is {_dr['name']}: a SHUT DOOR on first contact -> walking to it"))
                            elif (_thru_dr is not None
                                  and t - door_tried_at.get(_thru_dr["name"], -99.0) > 20.0):
                                door_tried_at[_thru_dr["name"]] = t
                                _door_scout = dict(mode="door", stage="thru",
                                                   t0=t, door=_thru_dr,
                                                   cell=cell_of(_thru_dr["x"], _thru_dr["y"]))
                                door_cool = t + 4.0
                                events.append((round(t,1), f"...that is {_thru_dr['name']} standing OPEN -- the lidar beats the floor plan, walking THROUGH it"))
                    scout = _door_scout
                    scan_cool = t + 8.0
                    replan_at = 0.0
                    mind.path = []
            # ---- GIMBAL SCAN (Bill: feet PLANTED — no marching in place — and
            # lean to his fullest, the July NODSCAN pass-2 radius of 20 deg).
            # Physics note: the lintel fingerprint lives in the SHALLOW dip
            # band (-4..-9 deg, where a dipped ray reaches 0.5-1.1 m before
            # the floor); a full-circle sweep carries every bearing through
            # every elevation, so we lean 20 and read the band that can see.
            elif scout is not None:
                A = math.radians(20.0); T_g = 16.0
                ph = 2*math.pi*(t - scout["t0"])/T_g
                drv.v = drv.vx = 0.0; drv.omega = 0.0
                drv.lift, drv.height = 0.0, cr["height"]
                drv.body_pitch = A*math.cos(ph); drv.body_roll = A*math.sin(ph)
                if t - scout["t0"] > 1.0:
                    for b in range(24):
                        az = math.radians(15*b)
                        elev = drv.body_pitch*math.cos(az) - drv.body_roll*math.sin(az)
                        if -0.16 < elev < -0.07:
                            scout["down"][b] = max(scout["down"][b], ring[b])
                        elif 0.07 < elev < 0.16:
                            # mirror of the down band: at steeper raises the
                            # ray clears the 0.30 wall TOP beyond ~0.5 m and
                            # fakes "no lintel" (up=2.0 killed true verdicts)
                            scout["up"][b] = max(scout["up"][b], ring[b])
                        elif abs(elev) < 0.03:
                            scout["level"][b] = max(scout["level"][b], ring[b])
                if t - scout["t0"] > 1.0 + T_g:
                    drv.body_pitch = drv.body_roll = 0.0
                    best_b, best_gap = None, 0.0
                    mxg = bx - math.sin(yawr)*MAST_FWD
                    myg = by + math.cos(yawr)*MAST_FWD
                    for b in range(24):
                        gap = scout["down"][b] - scout["up"][b]
                        # a tunnel is a low opening THROUGH A WALL: the level
                        # ray must be short too, or an open corridor's far
                        # floor fakes the fingerprint (false carves at 240/300)
                        if not (scout["level"][b] < 0.50 and scout["down"][b] > 0.55
                                and scout["up"][b] < 0.6 and gap > 0.30 and gap > best_gap):
                            continue
                        az = math.radians(15*b)
                        # same solid-span guard as the peek (edge artifacts)
                        if all(mind.expected_range(mxg, myg,
                                                   math.sin(az + da - yawr),
                                                   math.cos(az + da - yawr)) < 0.75
                               for da in (-0.10, 0.0, 0.10)):
                            best_gap, best_b = gap, b
                    mx = bx - math.sin(yawr)*MAST_FWD; my = by + math.cos(yawr)*MAST_FWD
                    scanned_pts.append((bx, by))     # the STANDPOINT, to match plan_scan_spot
                    if best_b is not None:
                        az = math.radians(15*best_b)
                        scout = dict(mode="confirm", t0=t, az=az, bb=best_b,
                                     ex=None, exp=None, src="gimbal")
                        events.append((round(t,1), f"GIMBAL: LOW OPENING at body-bearing {15*best_b} deg -> STAND-UP CHECK"))
                    else:
                        # DOOR REFLEX (Bill's wishlist): before giving up on
                        # a wall that failed to explain itself, SHOVE IT.
                        # Nothing in his world ever yielded to force until
                        # doors existed; now every dead-end wall gets one
                        # firm push as part of the scan ritual. Cheap on
                        # solid walls (3.5 s), transformative on doors.
                        events.append((round(t,1), "GIMBAL: nothing low -> trying a PUSH before moving on"))
                        scout = dict(mode="push", t0=t, x0=bx, y0=by)
            else:
                # ---- sense into the map (not on slopes: phantom walls
                # and LOW ceilings are what a level ring reads in a hill) ----
                if not slope_hold:
                    mind.sense(bx, by, yawr, rmin, rmax, ok, planted,
                               tall=not (t < crouch_until))
                posbuf.append((bx, by))
                moved = (len(posbuf) == posbuf.maxlen and
                         (bx-posbuf[0][0])**2 + (by-posbuf[0][1])**2 > 0.012**2)
                if (t < crouch_until and abs(roll) + abs(pitch) < 0.06
                        and (t < measure_until or planted or moved)
                        and float(data.body("base").xpos[2]) < 0.036):
                    # the z gate is PHYSICAL crouch, not commanded: during the
                    # 0.4 s drop the ring passes lintel height and clips face
                    # hits INSIDE the opening span (2.5 cm of south bias in
                    # the bracket from a handful of transition samples)
                    # crouched AND level AND actually moving (or deliberately
                    # planted measuring): fold the ring into any nearby
                    # carve's edge bracket. Level gate: wedge moments pitch
                    # the body and floor/lintel clips land INSIDE the opening
                    # span. Motion gate: while WEDGED, a pinned leg holds a
                    # persistent return the window max can't erase — that
                    # poisoned the bracket with phantom flank hits.
                    mind.refine_crouched(bx - math.sin(yawr)*MAST_FWD,
                                         by + math.cos(yawr)*MAST_FWD,
                                         yawr, wmax)
                    if os.environ.get("EDGE_DBG") and mind.carve_geo and int(t*50) % 25 == 0:
                        _nok = sum(1 for v in wmax if v is not None)
                        print(f"EDBG crouch t={t:.1f} pos=({bx:.2f},{by:.2f}) yaw={yawr:.2f} "
                              f"om={drv.omega:+.2f} wok={_nok} edge={mind.carve_edge[-1]}")
                if t < 0.8:
                    drv.v = drv.vx = 0.0; drv.omega = 0.0   # warm the map up planted
                else:
                    # (a front-blocked+map-free recognizer lived here and is
                    # REMOVED: its route gate was a catch-22 — the map marks
                    # the door WALL, the route dies, no route = no trigger —
                    # and it phantom-fired at non-doors. Recognition is now
                    # MAP-LEVEL: loaded-memory FREE cells that turn WALL are
                    # suspects, sought out at impasse. See door_next.)
                    # ---- DEADMAN (displacement, not commands) ----
                    # A PD pirouette with fwd gated to 0 evades BOTH watchdogs:
                    # the stuck detector needs commanded v, the pivot-lock
                    # watchdog needs a blocked front. One run spent 650 s
                    # silently spinning at the mouth corner between them. This
                    # one only asks: has the BODY moved? 20 s of no -> recover.
                    if last_move_xy is None or (bx-last_move_xy[0])**2 + \
                            (by-last_move_xy[1])**2 > 0.06**2:
                        last_move_xy = (bx, by)
                        last_move_t = t
                    elif t - last_move_t > 20.0:
                        deadman_hist = [dt_ for dt_ in deadman_hist
                                        if t - dt_ < 70.0] + [t]
                        if len(deadman_hist) >= 2:
                            # a REPEAT deadman = the plain back-off did
                            # nothing: he is physically HUNG (frames:
                            # high-centered on a door panel, belly on
                            # the edge). UNTANGLE: go TALL (push the
                            # belly off), big-lift steps, climb off
                            # BACKWARD; then alternate a tall strafe.
                            events.append((round(t,1), f"DEADMAN again at ({bx:+.2f},{by:+.2f}) -> I'm HUNG on something: UNTANGLE"))
                            untangle_until = t + 4.2
                            untangle_ph = untangle_n % 2
                            untangle_side = -untangle_side
                            untangle_n += 1
                            back_until = -1.0
                            replan_at = t + 4.4
                            if len(deadman_hist) >= 3:
                                # the SAME pocket keeps recapturing him:
                                # this is a planning ATTRACTOR (a stale
                                # lane pressing him into the wall), not
                                # geometry. Poison the whole local
                                # pocket and turn hard away -- break
                                # the basin, not just the cell.
                                events.append((round(t,1), "...the same pocket keeps taking me: POISON it and turn away"))
                                for _di7 in range(-1, 2):
                                    for _dj7 in range(-1, 2):
                                        _c7 = (here[0]+_di7, here[1]+_dj7)
                                        if not (in_grid(_c7) and _c7 != here
                                                and mind.grid[_c7] != LOW):
                                            continue
                                        if any(abs(_c7[0]-cell_of(_d8["x"], _d8["y"])[0]) <= 1
                                               and abs(_c7[1]-cell_of(_d8["x"], _d8["y"])[1]) <= 1
                                               for _d8 in door_reg):
                                            # NEVER poison a doorway --
                                            # the basin-break was
                                            # bricking the spawn room's
                                            # only exit with its own
                                            # escape hatch
                                            continue
                                        mind.bump[_c7] = min(
                                            mind.bump[_c7] + WALL_EVID, 24)
                                untangle_until = t + 7.0
                                untangle_ph = 2
                                replan_at = t + 7.2
                                deadman_hist = []
                        else:
                            events.append((round(t,1), f"DEADMAN: no displacement for 20 s at ({bx:+.2f},{by:+.2f}) -> recover"))
                            back_until = t + 1.6
                            replan_at = t + 1.6
                        if mind.goal is not None and not race:
                            mind.banned[mind.goal] = t + BAN_S
                            result["bans"] += 1
                            mind.goal = None
                        # RACE MODE HAS NO GOAL BAN -- so also poison
                        # the cell he is pressing into (the pivot-lock
                        # trick), or the same plan re-parks him in the
                        # same corner forever (frames: nose-parked at
                        # the mvB/south-wall pocket, untangle extracts,
                        # plan drives him straight back in)
                        _fx7 = bx - math.sin(yawr) * 0.14
                        _fy7 = by + math.cos(yawr) * 0.14
                        _fc7 = cell_of(_fx7, _fy7)
                        if in_grid(_fc7) and _fc7 != here and mind.grid[_fc7] != LOW:
                            mind.bump[_fc7] = min(mind.bump[_fc7] + WALL_EVID, 24)
                        mind.path = []
                        pivot_dir = 0.0
                        pivot_since = None
                        stuck_hist.clear()
                        last_move_t = t
                    # ---- stuck? (commanded motion, no displacement) ----
                    if heading is not None and abs(wrap(yawr - heading)) > 0.7:
                        stuck_hist.clear()      # a deliberate pivot is not stuck
                    stuck_hist.append((t, bx, by))
                    while stuck_hist and stuck_hist[0][0] < t - 8.0:
                        stuck_hist.popleft()
                    if (t > back_until and len(stuck_hist) > 2
                            and t - stuck_hist[0][0] > 7.5 and abs(drv.v) > 0.004
                            and (bx-stuck_hist[0][1])**2 + (by-stuck_hist[0][2])**2 < 0.05**2):
                        near_low = (not slope_on) and (
                            mind.grid[here] == LOW or any(
                                mind.grid[c] == LOW
                                for c in ((here[0]+dx, here[1]+dy) for dx, dy in N4)
                                if in_grid(c)))
                        if near_low:
                            # jammed at or inside a carved lane. A TRUE lane
                            # is physically marginal (0.34 m tunnel vs 0.29
                            # stance) so it jams too — the discriminator is
                            # eventual PROGRESS: strikes only count at the
                            # same spot, and a lane that never moves him a
                            # centimeter in 5 tries is a false verdict.
                            if grind_at and (bx-grind_at[0])**2 + (by-grind_at[1])**2 < 0.06**2:
                                grind_n += 1
                            else:
                                grind_n, grind_at = 1, (bx, by)
                            if grind_n >= 8 and mind.revoke_carves(bx, by):
                                events.append((round(t,1), f"lane never admits me at ({bx:+.2f},{by:+.2f}) -> verdict revoked"))
                                crouch_until = -1.0
                                grind_n, grind_at = 0, None
                            else:
                                events.append((round(t,1), f"lane jam at ({bx:+.2f},{by:+.2f}) -> push on ({grind_n})"))
                                crouch_until = t + 4.0
                                # STRAFE JITTER: he's a hexapod — re-approach
                                # the marginal tunnel on a shifted line
                                # instead of replaying the same jam
                                jitter = 0.006 if grind_n % 2 else -0.006
                        # BUMP INFERENCE: 8 s of commanded forward with no
                        # displacement IS a wall, whatever the lidar says.
                        # Mark the cell just ahead, and every bearing showing
                        # persistent inside-the-shell contact (the ring knows
                        # exactly where he's wedged); decay forgives errors.
                        fx = bx - math.sin(yawr)*0.14; fy = by + math.cos(yawr)*0.14
                        fc = cell_of(fx, fy)
                        repeat = any((bx-px)**2 + (by-py)**2 < 0.25**2
                                     for pt, px, py in stuck_pts if t - pt < 180.0)
                        stuck_pts.append((t, bx, by))
                        # width-as-suspicion + contact-as-proof: stuck at a
                        # spot the map calls one-cell-narrow = "I don't fit",
                        # first strike (no more long sieges of the E-D slot)
                        if not repeat and in_grid(fc) and                                 (mind.narrow(fc) or mind.narrow(here)):
                            repeat = True
                        if in_grid(fc) and fc != here and not near_low                                 and mind.grid[fc] != LOW:
                            # (never bump-block a crouch lane or its mouth)
                            mind.bump[fc] = min(mind.bump[fc] + WALL_EVID, 24)
                            if repeat:
                                # SECOND failed attempt at the same spot: he
                                # doesn't fit. Block the whole MOUTH (ahead
                                # cell + both perpendicular neighbors, heavy)
                                # instead of nibbling attempt after attempt.
                                pxu, pyu = math.cos(yawr), math.sin(yawr)
                                for off in (-CS, CS):
                                    cc = cell_of(fx + pxu*off, fy + pyu*off)
                                    if in_grid(cc) and mind.grid[cc] != LOW:
                                        mind.bump[cc] = min(mind.bump[cc] + 2*WALL_EVID, 24)
                                mind.bump[fc] = 24
                                events.append((round(t,1), f"I DON'T FIT at ({bx:+.2f},{by:+.2f}) -> opening blocked"))
                        mxx = bx - math.sin(yawr)*MAST_FWD
                        myy = by + math.cos(yawr)*MAST_FWD
                        for b in range(24):
                            h = ring_hist[b]
                            if len(h) == RING_WIN and max(h) < SELF_MIN:
                                az = math.radians(15*b)
                                ddx, ddy = math.sin(az - yawr), math.cos(az - yawr)
                                cc = cell_of(mxx + ddx*0.10, myy + ddy*0.10)
                                if in_grid(cc) and cc != here:
                                    mind._wall_evidence(cc, WALL_EVID)
                        if front < 0.35 and mind.goal is not None:
                            mind.banned[mind.goal] = t + BAN_S
                            result["bans"] += 1
                            mind.goal = None
                        events.append((round(t,1), f"STUCK at ({bx:+.2f},{by:+.2f}) front={front:.2f} -> bump-marked ahead, back off"))
                        mind.path = []
                        back_until = t + 1.4
                        replan_at = t + 1.4
                        stuck_hist.clear()

                    # ---- replan ----
                    _cov = mind.coverage()
                    if _cov != stag_cov:
                        stag_cov, stag_t = _cov, t
                    elif (t - stag_t > 55.0 and t > stag_leave_until
                          and not race):
                        # (race minds arrive COMPLETE: frozen coverage
                        # is their natural state, not a trap -- the
                        # escape marched a memory run to arbitrary
                        # corners for 420 s)
                        # STAGNANT: the map has not grown in 55 s -- he
                        # is trapped in a local carousel (false-lane
                        # scan/carve/revoke). Don't out-think it: LEAVE.
                        # March to the farthest reachable ground and let
                        # fresh geometry reset the hunt.
                        _dst9, _prv9, _ = mind.dijkstra(here)
                        _far9 = max(_dst9, key=_dst9.get, default=None)
                        if _far9 is not None and _dst9[_far9] > 3:
                            events.append((round(t,1), f"STAGNANT (map frozen at {_cov} cells) -> LEAVING this neighborhood for {_far9}"))
                            mind.path = mind._trace(_prv9, _far9)
                            mind.goal = None
                            scout = None
                            scan_next = False
                            scan_cool = t + 12.0
                            stag_leave_until = t + 25.0
                            stag_t = t
                            replan_at = t + 12.0
                    mid_pivot = (heading is not None and mind.path
                                 and abs(wrap(yawr - heading)) > 0.9)
                    if (t >= replan_at or not mind.path) and not mid_pivot \
                            and t >= idle_until:
                        if door_reg:
                            sync_doors()
                        old_goal = mind.goal
                        # a goal that never resolves is a phantom: ban it
                        # (not in race mode — the exit is not a phantom;
                        # stuck/impasse machinery handles race failures)
                        _gnear = False
                        if old_goal is not None:
                            _gw = world_of(old_goal)
                            _gnear = math.hypot(_gw[0] - bx, _gw[1] - by) < 0.24
                        if old_goal is not None and not race and (
                                t - goal_since > GOAL_MAX_HOLD_S
                                or t - goal_progress_t > 12.0
                                or (_gnear and t - goal_since > 2.5)):
                            # arm's-length rule: a frontier that stays
                            # unresolved from 15 cm away will not resolve
                            # from 5 -- resolve it NOW, don't orbit it
                            mind.banned[old_goal] = t + BAN_S
                            result["bans"] += 1
                            mind.goal = None
                            events.append((round(t,1), f"goal {old_goal} unresolved / no progress -> banned"))
                        # COMMIT: while the goal is a live frontier, only re-path
                        # to it — re-opening the frontier beauty contest every
                        # 2 s is what whipsawed him into a pirouette
                        path = None
                        if race:
                            path = mind.plan_exit(here)
                            if path is not None and len(path) >= 2:
                                mind.goal = path[-1]
                        if path is None and mind.goal_valid(t):
                            path = mind.plan_to(here, mind.goal)
                            if path is not None and len(path) < goal_best_d:
                                goal_best_d = len(path)
                                goal_progress_t = t
                        if path is None or len(path) < 2:
                            path = mind.plan_frontier(here, bx, by, yawr, t)
                            goal_best_d = 10**9
                            goal_progress_t = t
                            goal_since = t      # any re-selection is a fresh commitment
                        if path is not None and len(path) >= 2:
                            scan_next = False
                            door_next = None
                            mind.path = path
                        else:
                            # map exhausted: interrogate the nearest unexplained wall
                            mind.goal = None
                            if was_free is not None and t > door_cool and door_next is None:
                                _shield = set(door_banned)
                                for _ds in door_reg:
                                    # A CONQUERED DOOR IS NEVER A MEMORY
                                    # SUSPECT. He has been through it; if
                                    # his own traffic swings it shut, the
                                    # curiosity + sweep path reopens it.
                                    # The earlier 'shield only while
                                    # open' rule made the shield flicker
                                    # as the panel drifted 35..64 deg,
                                    # and he re-commuted to his own entry
                                    # door for 200 s of RUN 3's clock.
                                    if (_ds["name"] in door_done_names
                                            and abs(math.degrees(float(data.qpos[_ds["adr"]]))) >= 50.0):
                                        _shield.add(cell_of(_ds["x"], _ds["y"]))
                                _sus = [s for s in np.argwhere(was_free & (mind.grid == WALL))
                                        if (int(s[0]), int(s[1])) not in door_unreachable
                                        and not any(abs(int(s[0]) - _c0) <= 2
                                                    and abs(int(s[1]) - _c1) <= 2
                                                    for _c0, _c1 in _shield)]
                                if door_reg:
                                    # A DOOR SUSPECT NEEDS A DOOR. The
                                    # bare was_free/WALL discrepancy was
                                    # the recognizer back when the world
                                    # had one movable; in the vault a
                                    # saved mind carries dozens of stale
                                    # discrepancies (panel phantoms, his
                                    # own bump marks) and he chased map
                                    # GHOSTS across the maze all run --
                                    # 'never reached the door' every 40 s.
                                    # September equivalent: require a
                                    # door-like signature (hinge slit /
                                    # panel line), not just a map edit.
                                    _dcs = [cell_of(_d["x"], _d["y"]) for _d in door_reg]
                                    _sus = [s for s in _sus
                                            if any(abs(int(s[0]) - _c0) <= 2
                                                   and abs(int(s[1]) - _c1) <= 2
                                                   for _c0, _c1 in _dcs)]
                                if len(_sus):
                                    _dst, _prv, _ = mind.dijkstra(here)
                                    _bc, _bd = None, 1e18
                                    for _si, _sj in _sus:
                                        for _di, _dj in ((1,0),(-1,0),(0,1),(0,-1)):
                                            _n = (int(_si)+_di, int(_sj)+_dj)
                                            if _n in _dst and _dst[_n] < _bd:
                                                _bd, _bc = _dst[_n], (_n, (int(_si), int(_sj)))
                                    if _bc is not None:
                                        mind.path = mind._trace(_prv, _bc[0])
                                        door_next = _bc[1]
                                        door_next_t = t
                                        events.append((round(t,1), f"the map REMEMBERS an open passage at {door_next} -> going to try it"))
                            if door_next is None and door_reg and t > door_cool:
                                # DOOR CURIOSITY: boxed in (map exhausted)
                                # with a registered door-seam nearby and
                                # shut -- go KNOCK on it before wandering
                                # the room again (Bill watched 145 s of
                                # wall-pushing overture before the peek
                                # geometry happened to read the slit)
                                _dst3, _prv3, _ = mind.dijkstra(here)
                                _bd3, _bc3, _bdr3, _bopen3 = 1e18, None, None, False
                                for _dr in door_reg:
                                    _isopen = abs(math.degrees(float(data.qpos[_dr["adr"]]))) > 40.0
                                    # done doors that swung shut again
                                    # ARE knock candidates (the ladder
                                    # routes them to the sweep-reopen)
                                    if _isopen:
                                        # cross an open door only if the
                                        # FAR side holds unexplored map
                                        # (without this he ping-ponged
                                        # through his own conquered door
                                        # every cooldown, both ways)
                                        _hdf = _dr["hd"]
                                        _fwx = -math.sin(_hdf)
                                        _fwy = math.cos(_hdf)
                                        _sside = ((bx - _dr["x"]) * _fwx
                                                  + (by - _dr["y"]) * _fwy)
                                        _sgn5 = -1.0 if _sside > 0 else 1.0
                                        _fcx = _dr["x"] + _sgn5 * _fwx * 0.30
                                        _fcy = _dr["y"] + _sgn5 * _fwy * 0.30
                                        _fc5 = cell_of(_fcx, _fcy)
                                        _unk5 = 0
                                        for _di5 in range(-2, 3):
                                            for _dj5 in range(-2, 3):
                                                _c5 = (_fc5[0]+_di5, _fc5[1]+_dj5)
                                                if in_grid(_c5) and mind.grid[_c5] == UNK:
                                                    _unk5 += 1
                                        if _unk5 == 0 and not (race and _fcx > bx + 0.05):
                                            # far side worthless: no
                                            # unexplored map AND (in
                                            # race) not toward the
                                            # finish. Known-FREE beyond
                                            # a sealed gap is still
                                            # worth crossing eastward
                                            # (Bill: 'it shows empty
                                            # blocks on the other side')
                                            continue
                                    _dc3 = cell_of(_dr["x"], _dr["y"])
                                    if any(abs(_dc3[0]-_c0) <= 2 and abs(_dc3[1]-_c1) <= 2
                                           for _c0, _c1 in door_banned):
                                        continue
                                    if t - door_tried_at.get(_dr["name"], -99.0) < 30.0:
                                        continue
                                    for _di3 in range(-3, 4):
                                        for _dj3 in range(-3, 4):
                                            _n3 = (_dc3[0]+_di3, _dc3[1]+_dj3)
                                            if _n3 not in _dst3:
                                                continue
                                            # RADIUS search, not strict
                                            # adjacency: the swung panel
                                            # paints WALL across its own
                                            # gap, so the cells NEXT to
                                            # an open door are often
                                            # unreachable on the map --
                                            # walk to the closest cell
                                            # the map allows; the thru-
                                            # walk steers by geometry
                                            # for the sealed last meter.
                                            # Open doors are FREE
                                            # passages: strongly
                                            # preferred over knocks.
                                            _cost3 = (_dst3[_n3]
                                                      + 3 * (abs(_di3) + abs(_dj3))
                                                      - (1000 if _isopen else 0))
                                            if _cost3 < _bd3:
                                                _bd3, _bc3 = _cost3, _n3
                                                _bdr3, _bopen3 = _dr, _isopen
                                if _bc3 is not None:
                                    mind.path = mind._trace(_prv3, _bc3)
                                    door_next = cell_of(_bdr3["x"], _bdr3["y"])
                                    door_tried_at[_bdr3["name"]] = t
                                    door_visits[_bdr3["name"]] = door_visits.get(_bdr3["name"], 0) + 1
                                    if _bopen3:
                                        # his ring paints the swung panel
                                        # as a wall JUST past the gap --
                                        # the map seals the doorway with
                                        # the door (Bill: 'i guess its
                                        # hard for him to see the open
                                        # door?'). It's OPEN: walk to it
                                        # and surge through.
                                        scout = dict(mode="door", stage="thru",
                                                     t0=t, door=_bdr3,
                                                     cell=door_next)
                                        door_next = None
                                        events.append((round(t,1), f"boxed in -- {_bdr3['name']} is standing OPEN -> walking through it"))
                                    else:
                                        events.append((round(t,1), f"boxed in -- that seam is a DOOR ({_bdr3['name']}) -> going to knock"))
                            if door_next is not None and t - door_next_t > 40.0:
                                # STALE COMMUTE: 'en route' is a trance --
                                # while door_next is set the decision
                                # system waits politely, and a commute
                                # that can't arrive waited 419 s (RUN 2:
                                # one event in the whole run). Release.
                                _nd5 = None
                                if door_reg:
                                    # target the door this COMMUTE was
                                    # for -- 'nearest' kept sending him
                                    # back to the entry door behind him
                                    # (RUN 3 re-conquered nvd1 all run)
                                    for _d5 in door_reg:
                                        _c5d = cell_of(_d5["x"], _d5["y"])
                                        if (abs(_c5d[0] - door_next[0]) <= 2
                                                and abs(_c5d[1] - door_next[1]) <= 2):
                                            _nd5 = _d5
                                            break
                                    if _nd5 is not None:
                                        _open5 = abs(math.degrees(float(data.qpos[_nd5["adr"]]))) > 40.0
                                        if (_door_is_banned(_nd5)
                                                or (_nd5["x"]-bx)**2 + (_nd5["y"]-by)**2 > 1.30**2
                                                or (_open5 and _nd5["name"] in door_done_names)):
                                            _nd5 = None   # too far, or a
                                            # trophy door already standing open
                                if _nd5 is not None:
                                    # THE MAP CAN'T ROUTE ME, BUT I KNOW
                                    # WHERE IT IS: walk at the door by
                                    # GEOMETRY (the same doctrine that
                                    # beat the floor plan at the exit).
                                    # Commutes kept failing because the
                                    # door's own panel walls its cells.
                                    _hd5 = _nd5["hd"]
                                    if ((_nd5["x"] - bx) * -math.sin(_hd5)
                                            + (_nd5["y"] - by) * math.cos(_hd5)) < 0:
                                        _hd5 = wrap(_hd5 + math.pi)
                                    _isopen5 = abs(math.degrees(float(data.qpos[_nd5["adr"]]))) > 40.0
                                    events.append((round(t,1), f"can't route to {_nd5['name']} -> walking at it by GEOMETRY"))
                                    scout = dict(mode="door",
                                                 stage=("thru" if _isopen5 else "approach"),
                                                 t0=t, x0=bx, y0=by,
                                                 cell=cell_of(_nd5["x"], _nd5["y"]),
                                                 hd=_hd5, door=_nd5)
                                else:
                                    events.append((round(t,1), f"never reached the door at {door_next} -> giving up on that one"))
                                    door_unreachable.add(door_next)
                                door_next = None
                                door_cool = t + 10.0
                                replan_at = 0.0
                                mind.path = []
                            if door_next is not None:
                                pass                     # en route to the door
                            elif scan_next and t > scan_cool:
                                scout = dict(t0=t, mode="peek", down=[0.0]*24, up=[0.0]*24, level=[0.0]*24)
                                result["scouts"] += 1
                                scan_next = False
                                events.append((round(t,1), f"map exhausted [{mind.coverage()} cells] -> GIMBAL SCAN at ({bx:+.2f},{by:+.2f})"))
                            else:
                                spot = mind.plan_scan_spot(here, scanned_pts,
                                                           thorough=survey)
                                if spot is not None:
                                    mind.path = spot
                                    scan_next = True
                                    if len(spot) < 2:
                                        # already standing at the suspect wall
                                        mind.path = []
                                elif survey:
                                    result.update(done=True, reason="survey-complete")
                                    events.append((round(t,1), f"SURVEY COMPLETE [{mind.coverage()} cells]"))
                                    break
                                elif race and t > doubt_cool:
                                    # boxed in a "fully known" maze: the world
                                    # contradicts the map -> doubt everything
                                    n = mind.doubt()
                                    doubt_cool = t + 20.0
                                    replan_at = 0.0
                                    scanned_pts.clear()
                                    events.append((round(t,1), f"IMPASSE: map says no way out -> DOUBT ({n} walls demoted to unknown)"))
                                else:
                                    scanned_pts.clear()   # all walls tried: start over
                                    replan_at = t + 3.0
                                    idle_until = t + 3.0  # empty path bypasses
                                    # replan_at — without this hold the branch
                                    # fired every tick (an event flood)
                                    events.append((round(t,1), "every wall scanned, nothing -> rescanning"))
                        replan_at = max(replan_at, t + tn["replan"])

                    # ---- follow the path ----
                    if mind.path:
                        bj, bd = 0, 1e18
                        for j, c in enumerate(mind.path[:6]):
                            wx, wy = world_of(c)
                            dd = (wx-bx)**2 + (wy-by)**2
                            if dd < bd:
                                bd, bj = dd, j
                        if bj > 0:
                            del mind.path[:bj]
                        if mind.path and bd < 0.13**2:
                            mind.path.pop(0)
                        if not mind.path:
                            # arrived
                            if door_next is not None:
                                _dw = world_of(door_next)
                                _hd = math.atan2(-(_dw[0]-bx), (_dw[1]-by))
                                scout = dict(mode="door", stage="face", t0=t,
                                             x0=bx, y0=by, cell=door_next,
                                             hd=_hd)
                                door_next = None
                            elif scan_next:
                                replan_at = max(scan_cool, t)
                            elif mind.goal is not None and mind.unk_neighbors(mind.goal) > 0:
                                # standing on it and it's still a frontier: phantom
                                mind.banned[mind.goal] = t + BAN_S
                                result["bans"] += 1
                                events.append((round(t,1), f"reached {mind.goal}, still unresolved -> banned"))
                                mind.goal = None
                                replan_at = 0.0
                            else:
                                replan_at = 0.0
                    if mind.path:
                        # lookahead carrot with LINE OF SIGHT on his own map —
                        # aiming 3 cells ahead blindly cut corners into wall
                        # edges at the tight passages
                        ci = 0
                        for j in range(min(tn["carrot"], len(mind.path)) - 1, 0, -1):
                            wx, wy = mind.waypoint(mind.path[j])
                            fat = (0.0 if mind.grid[mind.path[j]] == LOW
                                   else 0.10 + drv.v * drv.T / 2)   # swing reach grows with stride
                            if mind.los(bx, by, wx, wy, width=fat):
                                ci = j
                                break
                        # carrot RATCHET: when the path doubles back nearby,
                        # the prune flips between near-equidistant cells and
                        # the heading oscillates him into a standstill — never
                        # retreat the carrot while the old one still has LOS
                        if last_carrot in mind.path[:6]:
                            pj = mind.path.index(last_carrot)
                            if pj > ci:
                                wxp, wyp = mind.waypoint(mind.path[pj])
                                fat = (0.0 if mind.grid[mind.path[pj]] == LOW
                                       else 0.10 + drv.v * drv.T / 2)
                                if mind.los(bx, by, wxp, wyp, width=fat):
                                    ci = pj
                        last_carrot = mind.path[ci]
                        # crouch is the ENTIRE knee bill — crouch as late
                        # and briefly as possible: by DISTANCE to the mouth,
                        # and SHOOT FOR THE MIDDLE (Bill): stage on the lane
                        # centerline 0.25 m out, arrive square, drive the pipe
                        low_i = next((i for i, c in enumerate(mind.path[:6])
                                      if mind.grid[c] == LOW), None)
                        if os.environ.get("EDGE_DBG") and int(t*100) % 50 == 0 \
                                and (t < 8.0 or 12.0 < t < 30.0):
                            print(f"MDBG t={t:.2f} pos=({bx:.2f},{by:.2f}) yaw={yawr:+.2f} "
                                  f"onlane={mind.on_lane(bx,by)} crouch={t<crouch_until} "
                                  f"climb={t<climb_until} back={t<back_until} "
                                  f"low_i={low_i} path0={mind.path[:3]} goal={mind.goal}")
                        if slope_on:
                            # ON A SLOPE the level ring stares into the
                            # hill — LOW marks and lane logic run on
                            # compromised geometry (112 s of phantom
                            # crouch-crawling per hills run). No lane
                            # crouching while the servo says tilted;
                            # real tunnels re-earn verdicts on the flat.
                            pass
                        elif mind.grid[here] == LOW or mind.on_lane(bx, by):
                            crouch_until = t + 1.2
                            near_mouth = True
                            mouth_facing = True   # committed to the lane
                        elif low_i is not None:
                            lw = mind.lane_center(mind.path[low_i])
                            d_mouth = math.hypot(lw[0]-bx, lw[1]-by)
                            near_mouth = d_mouth < 0.55
                            nxt = next((c for c in mind.path[low_i+1:low_i+4]
                                        if mind.grid[c] == LOW), None)
                            if nxt is not None:
                                nw_ = mind.lane_center(nxt)
                                axn = math.hypot(nw_[0]-lw[0], nw_[1]-lw[1]) or 1e-6
                                ax_, ay_ = (nw_[0]-lw[0])/axn, (nw_[1]-lw[1])/axn
                            else:
                                axn = d_mouth or 1e-6
                                ax_, ay_ = (lw[0]-bx)/axn, (lw[1]-by)/axn
                            # NEVER enter the pipe misaligned (Bill: "he
                            # runs into the side of the tunnel exit"): gate
                            # on cross-track (<6 cm off the lane axis) AND
                            # heading (within ~17 deg of the axis) — else go
                            # to the staging point first, even if it's behind
                            cross = abs((bx-lw[0])*ay_ - (by-lw[1])*ax_)
                            axis_head = math.atan2(-ax_, ay_)
                            aligned = (cross < 0.10
                                       and abs(wrap(yawr - axis_head)) < 0.35)
                            mouth_facing = abs(wrap(yawr - axis_head)) < 0.7
                            if aligned and d_mouth < 0.12:
                                crouch_until = t + 1.2   # crouch AT the mouth
                                # (Bill: "he knows where the opening is — he
                                # can crouch at it"): the drop completes with
                                # the mast still ~4 cm clear of the lintel
                            # (staging point RETIRED — Bill: "knowing his
                            # geometry, get there in the shortest distance at
                            # the premium angle": the final-approach curve
                            # below replaces the walk-away-then-return zigzag)
                        # MOUTH MICRO-PEEK (measure, then commit): if the
                        # governing lane's edge bracket is still incomplete,
                        # drop for 0.9 s and read the opening head-on — flank
                        # hits both sides + pass-throughs nail the centerline
                        # before the entry commits. Fires from EITHER branch:
                        # a carve trail can reach far enough back that he
                        # steps onto the lane before the approach logic ran
                        # (the trigger used to live only in the elif and
                        # starved — he then held a bad line faithfully into
                        # the wall face). Gated on being close enough to the
                        # anchor for flank hits (<= 0.5 m hit range).
                        mouth_gi = None
                        if near_mouth:
                            gkm = mind.geo_for(bx, by)
                            if gkm is None and low_i is not None:
                                gkm = mind.geo_for(*mind.lane_center(mind.path[low_i]))
                            if gkm is not None:
                                mouth_gi = gkm[1]
                            if (gkm is not None and t >= measure_until
                                    and mouth_measured.get(gkm[0], 0) < 1):
                                # ONE measuring ritual per mouth, total (Bill:
                                # "he shouldn't have to check twice") — the
                                # scan's in-place measure usually consumes it;
                                # this approach peek only serves lanes whose
                                # carve arrived unmeasured, and the in-pipe
                                # bracket refinement is the safety net beyond.
                                _g = mind.carve_geo[gkm[1]]
                                _dg = math.hypot(bx - _g[0], by - _g[1])
                                _axh = math.atan2(-_g[2], _g[3])
                                if (_dg < 0.42 and abs(wrap(yawr - _axh)) < 0.6
                                        and not mind.bracket_ok(gkm[1])):
                                    mouth_measured[gkm[0]] = mouth_measured.get(gkm[0], 0) + 1
                                    # 1.6 s: the physical drop eats ~0.5 s
                                    # before the z gate admits samples
                                    measure_until = t + 1.6
                                    crouch_until = max(crouch_until, measure_until)
                                    events.append((round(t,1), "mouth micro-peek: measuring the opening edges before committing"))
                        # ---- target: carrot -> stage -> measured lane line
                        # (this ordering used to be inverted: stage was set
                        # AFTER the target consumed it and reset each tick —
                        # the staging override never actually fired)
                        if mind.grid[mind.path[ci]] == LOW:
                            wx, wy = mind.lane_center(mind.path[ci])
                        else:
                            wx, wy = mind.waypoint(mind.path[ci])
                        if (mouth_gi is not None and not (t < crouch_until)
                                and mind.bracket_ok(mouth_gi)):
                            # ... and only on a MEASURED line: flying an
                            # unverified carve at the premium angle delivers
                            # him precisely to the wrong spot (harness 0.62
                            # case thrashed 40 s). Until the one peek lands,
                            # the carrot flow carries him close enough for it.
                            # FINAL APPROACH (Bill: "knowing his geometry, get
                            # there in the shortest distance, at the premium
                            # angle"): aim at the measured line, lookahead
                            # stretching with lateral error — far off-line
                            # gives a shallow intercept (max ~29 deg, no
                            # oscillation at speed), close-in tightens for
                            # precision. Replaces the staging-point zigzag.
                            g3 = mind.carve_geo[mouth_gi]
                            off3 = mind.edge_offset(mouth_gi)
                            cr3 = (-(bx-g3[0])*g3[3] + (by-g3[1])*g3[2]
                                   - off3)
                            la3 = 0.22 + 1.8 * abs(cr3)
                            al3 = (bx-g3[0])*g3[2] + (by-g3[1])*g3[3] + la3
                            wx = g3[0] + g3[2]*al3 - g3[3]*off3
                            wy = g3[1] + g3[3]*al3 + g3[2]*off3
                        if t < crouch_until:
                            # (an approach-phase line-follow experiment was
                            # tried and REVERTED: pre-crouch following of the
                            # line at standing speed oscillates on the crawl-
                            # tuned lookahead and fights the stage/peek flow —
                            # entry alignment polish belongs to the genes)
                            gk2 = mind.geo_for(bx, by)
                            if gk2 is not None:
                                # LINE-FOLLOWING while crouched on a lane: aim
                                # at the measured line 0.16 m ahead. Crouched
                                # wave gait drifts laterally (clutch-slip
                                # asymmetry) and the distant carrot has almost
                                # no lateral authority — corridor-centering
                                # was the accidental drift corrector before it
                                # was suppressed here; this is the deliberate
                                # one, anchored to the measured line. The
                                # INTEGRAL term kills the 2-3 cm steady-state
                                # error a pure P-follower holds under drift.
                                on_line = True
                                g2 = mind.carve_geo[gk2[1]]
                                off2 = mind.edge_offset(gk2[1])
                                cr_s = (-(bx-g2[0])*g2[3] + (by-g2[1])*g2[2]
                                        - off2)
                                lane_int = float(np.clip(lane_int + cr_s*0.01,
                                                         -0.05, 0.05))
                                tgt_c = off2 - float(np.clip(1.2*lane_int, -0.05, 0.05))
                                al2 = (bx-g2[0])*g2[2] + (by-g2[1])*g2[3] + 0.16
                                wx = g2[0] + g2[2]*al2 - g2[3]*tgt_c
                                wy = g2[1] + g2[3]*al2 + g2[2]*tgt_c
                        if not on_line:
                            lane_int = 0.0
                        if math.hypot(wx - bx, wy - by) < 0.14 and heading is not None:
                            pass   # POINT-BLANK carrot: hold the last
                            # heading. atan2 to a point under his nose
                            # swings wildly with every step -- the PD
                            # chased a spinning carrot and he ORBITED a
                            # one-cell-away goal for 20 s (Bill, live:
                            # 'spinning his way into the first door,
                            # then spinning back into the room')
                        else:
                            heading = math.atan2(-(wx-bx), (wy-by))

                    # ---- drive ----
                    # SONAR CLIMB (obstacles inside corridors): a shortfall vs
                    # the HEIGHT-DEPENDENT floor baseline = a low bar the ring
                    # can't see (front stays open — that's the discriminator vs
                    # a wall). Posture from the graduated course policy: taller
                    # + higher lift, slower — and COMMIT by DISTANCE, not time
                    # (a 3 s timer expired mid-approach and parked him astride).
                    sonar_floor = (0.0403 + drv.height) / 0.1736
                    # a BAR is sonar-short with the ring truly open ahead —
                    # the 0.35 front gate let wall glances fake 40 s of false
                    # climbs per maze run at 79% knee sat (the tall posture
                    # is knee-brutal). Gate hard, commit short, release the
                    # moment the sonar sees clean floor again.
                    if front > 0.60 and 0.10 < sonar < sonar_floor - 0.05 \
                            and not (t < crouch_until) \
                            and mind.expected_range(
                                bx - math.sin(yawr)*MAST_FWD,
                                by + math.cos(yawr)*MAST_FWD,
                                -math.sin(yawr), math.cos(yawr)) > sonar * 0.985 + 0.12:
                        # the MAP as the bar-vs-wall-clip discriminator: if a
                        # mapped wall already explains the sonar shortfall,
                        # this is a clip, not a bar (32.7 s of false climb at
                        # 0.95 knee sat in one known-map race). Unmapped
                        # terrain keeps the old behavior (expected 2.0).
                        climb_arm += 1
                        if climb_arm >= 50:      # bar signature held 0.5 s —
                            climb_until = t + 25.0   # transient wall clips fail
                            if climb_from is None:
                                climb_from = (bx, by)
                    else:
                        climb_arm = 0
                    if t < climb_until and sonar > sonar_floor - 0.03:
                        climb_until = min(climb_until, t + 0.6)
                    if climb_from is not None and                             (bx-climb_from[0])**2 + (by-climb_from[1])**2 > 0.30**2:
                        climb_until = -1.0
                        climb_from = None
                    crouching = t < crouch_until
                    climbing = t < climb_until and not crouching
                    if not climbing:
                        climb_from = None
                    if t >= back_until:
                        jitter = 0.0
                    pk = dk if crouching else cr
                    # gait selection by posture: crouched knees are at the
                    # clutch limit under TRIPOD (3 stance legs) — WAVE puts 5
                    # legs under the body and shares the load
                    want_g = 1 if crouching else 0        # WAVE : TRIPOD
                    want_T = (0.911 if want_g == 1 else
                              (0.420 if zooming else tn["cadence"] / 1000.0))
                    if drv.g != want_g or abs(drv.T - want_T) > 1e-6:
                        # PHASE-CONTINUOUS gear shift: changing T rescales
                        # cyc=t/T and teleports the feet mid-stride (every
                        # zoom toggle was a stumble) — carry the phase over
                        _cyc = (drv.t / drv.T) % 1.0
                        drv.g = want_g
                        drv.beta = G.BETA[want_g]
                        drv.T = want_T
                        drv.t = _cyc * want_T
                    if climbing:
                        # course-policy posture: tall + high lift + SURGE —
                        # "momentum beats caution", never stall astride a bar
                        pk = dict(speed=cr["speed"]*1.05, lift=0.030, height=0.030)
                    drv.lift, drv.height = pk["lift"], pk["height"]
                    if on_line and crouching:
                        # under the lintel the swing apex at full duck lift
                        # (0.022) strikes it from BELOW — a gait-phase lottery
                        # that jammed random exits for eras. But too LOW a cap
                        # stalls him outright: at crouch depth the femurs press
                        # into his own chassis flanks (the anatomy behind the
                        # 40% crawl slip) and swing feet stop clearing. The
                        # harness swept the trade: EXP_PIPE_LIFT clears both.
                        drv.lift = min(drv.lift, EXP_PIPE_LIFT)
                    drv.vx = 0.0
                    drv.body_yaw = 0.0
                    if t < measure_until:
                        drv.v = drv.vx = 0.0        # planted micro-peek
                        drv.omega = 0.0
                        drv.lift = 0.0
                    elif t < back_until:
                        drv.v = -G.WALK_SPEED_CM_S * 0.01 * 0.5
                        drv.vx = jitter
                        drv.omega = 0.0
                    elif t < untangle_until:
                        # THE UNTANGLE: tall (belly off the edge), high
                        # lift, climb off backward -- or tall strafe on
                        # the alternate attempt
                        drv.height = 0.042
                        drv.lift = 0.026
                        drv.body_pitch = 0.0
                        drv.omega = 0.0
                        if untangle_ph == 0:
                            drv.v = -G.WALK_SPEED_CM_S * 0.01 * 0.55
                            drv.vx = 0.0
                        elif untangle_ph == 1:
                            drv.v = -G.WALK_SPEED_CM_S * 0.01 * 0.25
                            drv.vx = 0.024 * untangle_side
                        else:
                            # basin-break: back out then wheel hard away
                            if untangle_until - t > 4.0:
                                drv.v = -G.WALK_SPEED_CM_S * 0.01 * 0.55
                                drv.vx = 0.0
                            else:
                                drv.v = 0.0
                                drv.omega = 2.5 * untangle_side
                                drv.T = 0.38
                    elif t < surge_until:
                        # breach follow-through: straight out the door,
                        # full stride, no planning until he's CLEAR
                        _se = wrap(surge_hd - yawr)
                        drv.v = cr["speed"] * 0.035
                        drv.omega = float(np.clip(-1.8 * -_se, -0.42, 0.42))
                        drv.lift = cr["lift"]
                    elif heading is None or not mind.path:
                        drv.v = 0.0
                        drv.omega = 0.0
                    else:
                        yaw_err = wrap(yawr - heading)
                        drv.omega = float(np.clip(-tn["kyaw"]*yaw_err, -0.42, 0.42))
                        # CORNERING YAW LEAD (Bill: "the YAW... might even
                        # help in his cornering abilities"): pose-yaw the
                        # body INTO the turn while the gait still carves
                        # the track -- nose (and the whole ring) looks
                        # through the corner early. body_yaw rotates foot
                        # targets, the body counter-rotates: sign follows
                        # yaw_err. Off in the pipe (line-threading wants
                        # the body square).
                        drv.body_yaw = (0.0 if crouching else
                                        float(np.clip(tn.get("yaw_lead", 0.0) * yaw_err,
                                                      -0.32, 0.32)))
                        if (abs(yaw_err) < 0.5 and (left < 0.35 or right < 0.35)
                                and not near_mouth and not crouching):
                            # corridor centering (scout-mode trick): lean away
                            # from the nearer wall so the nose never grinds it.
                            # SUPPRESSED near lane mouths and in the pipe: the
                            # flanks read asymmetric there (D2 short north,
                            # open floor south) and the nudge drags him off
                            # the measured lane line.
                            drv.omega = float(np.clip(
                                drv.omega + np.clip(1.2*(right - left), -0.24, 0.24),
                                -0.42, 0.42))
                        # blocked handling with HYSTERESIS: engage under 0.17,
                        # release only above 0.25, and while engaged the pivot
                        # direction is COMMITTED — mixing the stale pivot with
                        # fresh PD as front flapped around one threshold parked
                        # him at wall corners for hundreds of seconds
                        # THE PUSHING DETECTOR: range pinned while the
                        # body travels = the thing ahead is MOVING WITH
                        # HIM (a door panel yielding). A pushed movable
                        # must not read as a wall: no pivot, keep
                        # driving, slate the phantom marks it paints.
                        if (front < 0.55 and drv.v > 0.01):
                            if (push_ref is None
                                    or abs(front - push_ref[2]) > 0.06):
                                push_ref = (bx, by, front)
                            elif math.hypot(bx - push_ref[0],
                                            by - push_ref[1]) > 0.12:
                                if t > push_until + 3.0:
                                    events.append((round(t,1), f"range pinned at {front:.2f} while I traveled -> I'm PUSHING something movable"))
                                    push_wide = None
                                push_until = t + 1.5
                                _door_slate(cell_of(
                                    bx - math.sin(yawr) * (front + 0.05),
                                    by + math.cos(yawr) * (front + 0.05)))
                                push_ref = (bx, by, front)
                                # WIDENING RECOGNITION (Bill: "he should
                                # recognize that opening getting wider
                                # too -- his actions are creating that
                                # opening"): count forward bins seeing
                                # PAST the pushed thing; a rising count
                                # while pushing = he is opening a
                                # passage -> carve it into the map with
                                # confidence (it is not a sensor ghost,
                                # it is HIS doing) so the planner routes
                                # through instead of hunting phantoms.
                                _wn = sum(1 for _b in (22, 23, 0, 1, 2)
                                          if wmax[_b] is not None
                                          and wmax[_b] > front + 0.25)
                                if push_wide is None:
                                    push_wide = _wn
                                elif _wn > push_wide:
                                    events.append((round(t,1), f"the opening is WIDENING ({push_wide}->{_wn} clear bearings) -- MY push is creating it"))
                                    push_wide = _wn
                                    for _b in (22, 23, 0, 1, 2):
                                        if wmax[_b] is None or wmax[_b] <= front + 0.25:
                                            continue
                                        _az = math.radians(15 * _b)
                                        _dxw = math.sin(_az - yawr)
                                        _dyw = math.cos(_az - yawr)
                                        _dw = 0.10
                                        while _dw < front + 0.40:
                                            _cw = cell_of(bx + _dxw * _dw,
                                                          by + _dyw * _dw)
                                            if in_grid(_cw) and mind.grid[_cw] != LOW:
                                                mind.evid[_cw] = min(mind.evid[_cw], -40)
                                                if mind.grid[_cw] in (WALL, UNK):
                                                    mind.grid[_cw] = FREE
                                            _dw += 0.075
                                        replan_at = 0.0
                        else:
                            push_ref = None
                            push_wide = None
                        blocked = front < (0.25 if pivot_dir != 0.0 else 0.17)
                        if t < push_until:
                            blocked = False    # it's yielding: drive on
                            if front > 0.60:
                                # PUSH COMPLETE (Bill: "he's got the door
                                # fully open, but is continuing to push
                                # on it, even though the opening is fully
                                # passable"): the pushed thing swung out
                                # of his arc -- the way is CLEAR. Stop
                                # pushing, route THROUGH it, now.
                                events.append((round(t,1), "PUSHED IT OPEN -- the way is passable, going through"))
                                push_until = t
                                push_ref = None
                                push_wide = None
                                _door_slate(here)
                                replan_at = 0.0
                                mind.path = []
                        if blocked or t < back_until:
                            zoom_arm, zooming = 0, False
                        if blocked and not crouching:
                            if pivot_since is None:
                                pivot_since = t
                            elif t - pivot_since > 12.0:
                                # a full revolution changed nothing: the pivot
                                # is LOCKED (corner reads 0.18-0.24 forever) —
                                # and with v=0 the stuck detector is blind to
                                # it. Bump-mark, back off, replan.
                                fx = bx - math.sin(yawr)*0.14
                                fy = by + math.cos(yawr)*0.14
                                fc = cell_of(fx, fy)
                                if in_grid(fc) and fc != here and mind.grid[fc] != LOW:
                                    mind.bump[fc] = min(mind.bump[fc] + WALL_EVID, 24)
                                events.append((round(t,1), f"pivot-locked at ({bx:+.2f},{by:+.2f}) -> bump-mark, back off"))
                                back_until = t + 1.6
                                replan_at = t + 1.6
                                mind.path = []
                                pivot_dir = 0.0
                                pivot_since = None
                            if pivot_dir == 0.0:
                                if abs(yaw_err) > 0.4:
                                    pivot_dir = math.copysign(0.38, -yaw_err)
                                else:
                                    pivot_dir = 0.38 if left > right else -0.38
                            drv.v = 0.0
                            drv.omega = pivot_dir
                            replan_at = min(replan_at, t + 0.5)
                        else:
                            pivot_dir = 0.0
                            pivot_since = None
                            # corner veer: a wall corner shows up in the
                            # forward DIAGONAL before it can reach the flank
                            # (the scout-mode junction lesson) — steer away
                            # so lane entries thread the middle, not the edge
                            veer = (0.0 if (crouching
                                            or (near_mouth and mouth_facing))
                                    else 0.22 * ((1 if rdiag < 0.20 else 0)
                                                 - (1 if ldiag < 0.20 else 0)))
                            # veer yields only when crouched or flying the
                            # aligned final approach — the old blanket
                            # near-mouth suppression also stripped corner
                            # protection from the OBLIQUE approach leg (Bill:
                            # "hung up on the wall/corner to the left of him
                            # coming back to the opening")
                            drv.omega = float(np.clip(drv.omega + veer, -0.42, 0.42))
                            ae = abs(yaw_err)
                            # ZOOMIES: a long open straight, heading settled,
                            # held for half a second -> shift up to the
                            # sustainable sprint tier. Any condition fails ->
                            # shift straight back down.
                            runway = wmax[0] if wmax[0] is not None else 0.0
                            # zoomies needs REAL runway: in tight mazes sprint
                            # bursts trade straight-line seconds for corner
                            # chaos (188s vs 154s measured) — 1.3 m keeps it
                            # an open-ground behavior
                            if (runway > 0.95 and front > 0.35 and ae < 0.12
                                    and not crouching
                                    and not climbing and not tilted
                                    and len(mind.path) >= 4):
                                zoom_arm += 1
                            else:
                                zoom_arm = 0
                            zooming = zoom_arm >= 30
                            fwd = 1.0 if ae < tn["fwd_stop"] else 0.0
                            # SLOW IN, FAST OUT: steering strokes stack on
                            # stride strokes at the knees (probe: hard steer
                            # at speed = 18% sat, straight overdrive = 2%) —
                            # and the stack is what dropped the fallen
                            # overdrive kids. Throttle follows the wheel.
                            om_d = abs(drv.omega)
                            if om_d > tn["brake_at"]:
                                fwd *= max(0.35, 1.0 - tn["brake_slope"]
                                           * (om_d - tn["brake_at"]))
                            if tilted:
                                fwd *= tn["tilt_slow"]   # pitfall care
                            spd = tn["zoom"] if zooming else pk["speed"]
                            base = G.WALK_SPEED_CM_S * 0.01 * spd * fwd
                            # HE IS A HEXAPOD, NOT A CAR (Bill: "stuck on the
                            # first turn for 15 seconds"): the velocity vector
                            # keeps flowing toward the carrot while the body
                            # yaws round — forward = cos(err), crab = sin(err).
                            # AND (Bill's choreography): "his head should find
                            # the corner, then his ass should swing around it"
                            # — the pivot point moves to the NOSE: vx += w*d
                            # swings the tail wide and strafes the body AWAY
                            # from the inside of the turn, while the carrot-
                            # crab (which pulls INTO the corner) fades out as
                            # the turn tightens. Middle legs clear the apex.
                            drv.v = base * max(math.cos(yaw_err), 0.0)
                            # MOMENTUM TRANSFER (Bill: "transfer his forward
                            # momentum into lateral, and then forward again —
                            # that was the point behind the drift/strafe"):
                            # crab gain is a GENE, the apex fade is softened
                            # from full-kill to half (it existed to keep
                            # middle legs off apexes — glances now cost in
                            # fitness, so evolution prices that trade), and
                            # the lateral channel cap doubles: at the apex the
                            # velocity vector lags the body instead of being
                            # dumped and rebuilt.
                            f_ap = min(1.0, abs(drv.omega) / 0.42)
                            crab = (G.WALK_SPEED_CM_S * 0.01 * spd
                                    * math.sin(yaw_err)
                                    * (0.7 * (1.0 - f_ap)
                                       + tn.get("crab", 0.0) * f_ap))
                            # two crab regimes: the legacy term lives at LOW
                            # omega (gentle curves) and dies at the apex —
                            # in 0.45 m corridors sustained lateral is poison
                            # (a flat 0.5-fade experiment ground 650 s of
                            # wall). The APEX term is Bill's momentum
                            # transfer — velocity lagging the body exactly
                            # when the turn is tightest — gene-gained from
                            # ZERO so evolution buys it only where the
                            # glance-priced fitness says it pays.
                            swing = 0.18 * drv.omega
                            drv.vx = float(np.clip(crab + swing, -0.028, 0.028))
                            # clip 0.028 is LOAD-BEARING: it calibrates the
                            # nose-pivot swing (0.18*omega = 0.076 raw at
                            # full turn) — raising it to 0.055 doubled every
                            # tail swing and ground 150-650 s of wall per run
                            if crouching:
                                # NO lateral moves in a tunnel (Bill: stuck
                                # halfway out the exit from ass-swinging) —
                                # 0.34 m lane, 0.29 m body: thread it straight
                                drv.vx = 0.0
        if map_cb is not None and s % (CTRL_EVERY * 50) == 0:
            map_cb(dict(grid=mind.grid.copy(), path=list(mind.path), goal=mind.goal,
                        x=float(data.body("base").xpos[0]),
                        y=float(data.body("base").xpos[1]),
                        yaw=yaw_of(data.body("base").xquat), t=t,
                        mode=("SCAN" if scout is not None else
                              ("GOTO-SCAN" if scan_next else "NAV")),
                        scans=list(scanned_pts), coverage=mind.coverage(),
                        # spectator extras (Bill: "so i'm not waiting for you
                        # to produce reels all the while clueless"):
                        carves=[(float(g[0]), float(g[1]), float(g[2]),
                                 float(g[3]), float(mind.edge_offset(gi)))
                                for gi, g in enumerate(mind.carve_geo)],
                        glances=list(glance_pts),
                        wall_hits=result["wall_hits"],
                        scan3d=scan3d_new))
            scan3d_new = []
        if s % CTRL_EVERY == 0 and t > 0.8:
            # WALKING ATTITUDE SERVO (balance-gym stage A, ported into the
            # gait): counter-pose the body so chassis and lidar ring stay
            # level on moving/sloped ground. Input is LOW-PASSED (tau 0.15)
            # so the servo ignores its own ~2.3 Hz stride bounce but tracks
            # the deck's slow roll. AXIS NOTE (the morning's lesson): this
            # file's aviation-named `roll` is rotation-about-x = the
            # driver's body_PITCH; `pitch` (about y) = body_ROLL.
            # Gated off during scans (deliberate poses), climbs (commanded
            # posture), and crouches (lintel margin) — poses decay there.
            _srv_ok = not (world_kw or {}).get("shortcut")
            # shortcut world = the tipping plate lives there: walking
            # servo off entirely (see the negative result below)
            if (_srv_ok and scout is None and not (t < climb_until)
                    and not (t < crouch_until)):
                att_fp += 0.067 * (roll - att_fp)
                att_fr += 0.067 * (pitch - att_fr)
                # HYSTERESIS ARMING — the servo engages only on SUSTAINED
                # tilt (deck, slope), never transient cornering lean:
                # always-on, it fought the co-adapted turn dynamics and
                # ground 164 wall glances into a clean 75 s panic run
                _tmag = abs(att_fp) + abs(att_fr)
                if os.environ.get("NO_ATT_SERVO"):
                    pass                         # A/B baseline switch
                elif not att_on:
                    att_arm = att_arm + 1 if _tmag > 0.035 else 0
                    if att_arm >= 50:            # 0.5 s persistent -> arm
                        att_on, att_arm = True, 0
                else:
                    att_arm = att_arm + 1 if _tmag < 0.015 else 0
                    if att_arm >= 150:           # 1.5 s calm -> disarm
                        att_on, att_arm = False, 0
                if att_on:
                    att_ip = att_ip * (1 - 0.58 * 0.01) + att_fp * 0.01
                    att_ir = att_ir * (1 - 0.58 * 0.01) + att_fr * 0.01
                    _tp = float(np.clip(-(1.5 * att_fp + 0.8 * att_ip), -0.22, 0.22))
                    _tr = float(np.clip(-(1.5 * att_fr + 0.8 * att_ir), -0.22, 0.22))
                    drv.body_pitch += (0.01 / 0.12) * (_tp - drv.body_pitch)
                    drv.body_roll += (0.01 / 0.12) * (_tr - drv.body_roll)
                    # (COM COMPANION EXPERIMENT — REMOVED, negative result
                    # worth keeping: on FREE-tilting ground the attitude
                    # servo alone pumps the tip (leaning level loads the
                    # downhill edge — it flipped him at the shortcut plate),
                    # and the stage-B COM pair fixes exactly that (159.7 s
                    # crossing) — but the pair WRECKS rigid moving ground
                    # (rocker 900 s: COM can't move a driven deck, it just
                    # wiggles the stance). FIVE IMU-only rigid-vs-free
                    # discriminators failed, most because the servo erases
                    # the very tilt signal being discriminated. Walking
                    # servo ships ATTITUDE-ONLY; free-tipping surfaces
                    # while WALKING are a documented open problem — the
                    # pair works standing (balance_gym stage B), and the
                    # shortcut world runs servo-free (its pre-servo state).
                    # See git history for the removed pair + discriminators.)
                else:
                    att_ip = att_ir = com_ip2 = com_ir2 = 0.0
                    drv.body_pitch *= 0.97
                    drv.body_roll *= 0.97
                    drv.body_dx *= 0.97
                    drv.body_dy *= 0.97
            elif scout is None:
                att_ip = att_ir = 0.0
                drv.body_pitch *= 0.97
                drv.body_roll *= 0.97
        if rock_adr is not None:
            # the deck is DRIVEN, not passive: the world moves under him on
            # its own schedule (kinematic forcing — infinitely stiff, his
            # weight doesn't argue with it; compliant give can come later)
            _rw = 2 * math.pi * 0.35
            data.qpos[rock_adr] = math.radians(3.2) * math.sin(_rw * t)
            data.qvel[rock_dof] = math.radians(3.2) * _rw * math.cos(_rw * t)
        data.ctrl[:] = drv.ctrl(model.opt.timestep)
        mujoco.mj_step(model, data)
        t += model.opt.timestep
        _bp = data.body("base").xpos
        if 2.00 < _bp[0] < FINISH_X:
            # audit window starts at 2.00: under the nose-at-wire finish the
            # body only reaches ~2.27, so the old 2.18 window caught 0.3 s of
            # single-blink statistics instead of the whole thread
            # feet filtered by their OWN x: only feet inside the wall band
            # face the 0.53/0.87 constraint (body-x filtering flagged legal
            # open-floor foot positions as wall violations)
            _f = data.geom_xpos[foot_gid]
            _in = np.abs(_f[:, 0] - 2.30) < 0.03
            _fyN = float(_f[_in, 1].max()) if _in.any() else np.nan
            _fyS = float(_f[_in, 1].min()) if _in.any() else np.nan
            tun.append((float(_bp[0]), float(_bp[1]), _fyN, _fyS))
        kf = np.abs(data.actuator_force[2::3])
        _hit = int((kf > G.SERVO_TORQUE * 0.95).any())
        sat_knee += _hit
        sat_n += 1
        _m = ("scan" if scout is not None else
              "crouch" if t < crouch_until else
              "back" if t < back_until else
              "climb" if t < climb_until else
              "pivot" if abs(drv.omega) > 0.30 else
              "steer" if abs(drv.omega) > 0.12 else "straight")
        _sm = sat_by.setdefault(_m, [0, 0])
        _sm[0] += _hit
        _sm[1] += 1
        if viewer is not None:
            if not viewer.is_running():
                break
            viewer.sync()
            lag = (wall0 + t) - time.perf_counter()
            if lag > 0:
                time.sleep(lag)
        if renderer is not None and s % 16 == 0:
            cam = mujoco.MjvCamera(); cam.lookat[:] = data.body("base").xpos
            if cam_ctl:      # spectator's mouse: orbit + zoom, still tracking
                cam.distance = cam_ctl.get("distance", 1.1)
                cam.elevation = cam_ctl.get("elevation", -50)
                cam.azimuth = cam_ctl.get("azimuth", 140)
            else:
                cam.distance, cam.elevation, cam.azimuth = 1.1, -50, 140
            renderer.update_scene(data, camera=cam)
            fr = renderer.render()
            if video:
                frames.append(fr.copy())
            if frame_cb is not None:
                frame_cb(fr)
                lag = (wall0 + t) - time.perf_counter()
                if lag > 0:          # real-time pacing for the spectator
                    time.sleep(lag)
        if (float(data.body("base").xpos[0])
                - math.sin(yawr) * MAST_FWD >= FINISH_MAST_X and not survey):
            # THE NOSE AT THE WIRE (Bill): finished when the LIDAR UNIT
            # passes the threshold plane — "that means he effectively got
            # out, he's just takin his sweet-ass time across the finish
            # line". The old rule wanted body center 10 cm past the face;
            # with the mast 5.5 cm ahead, this fires ~15 cm (~5-7 s of
            # crawl) earlier. Planning still targets FINISH_X cells beyond
            # the wall so routes never end inside the pipe.
            result.update(done=True, reason="finished")
            break
        if data.body("base").xpos[2] < (0.008 if (scout is not None
                and scout.get("mode") == "door") else 0.02):
            # the door ladder's deep rear legitimately sinks the body low —
            # a stunt is not a fall; only true belly-flat ends a ladder run
            result.update(reason="fell")
            break
    if viewer is not None:
        viewer.close()
    if save_mind:
        mind.save(save_mind)
    if tun:
        _ta = np.array(tun)
        # stats in the FEET-CONSTRAINED zone: feet reach +-0.12 from center,
        # so body x 2.16-2.34 is where the 0.53/0.87 faces actually bind.
        # (Was 2.26-2.34 = body inside the wall itself; under the
        # nose-at-wire finish the body stops at ~2.27 and that window
        # caught 0.3 s of single-blink statistics.)
        _pb = _ta[(_ta[:, 0] > 2.16) & (_ta[:, 0] < 2.34)]
        _src = _pb if len(_pb) else _ta
        with np.errstate(all="ignore"):
            _fN = np.nanmax(_ta[:, 2]) if np.isfinite(_ta[:, 2]).any() else np.nan
            _fS = np.nanmin(_ta[:, 3]) if np.isfinite(_ta[:, 3]).any() else np.nan
        result["tunnel"] = dict(
            y=[round(float(_src[:, 1].min()), 3), round(float(_src[:, 1].mean()), 3),
               round(float(_src[:, 1].max()), 3)],
            footN=(round(float(_fN), 3) if np.isfinite(_fN) else None),  # vs north face 0.87
            footS=(round(float(_fS), 3) if np.isfinite(_fS) else None),  # vs south face 0.53
            band_s=round(len(_pb) * model.opt.timestep, 1))
    if mind.demoted:
        result["low_demoted"] = mind.demoted
    if mind.carve_geo:
        # each carve: line anchor+axis, refined offset, bracket sample counts
        result["carves"] = [
            [round(float(g[0]), 2), round(float(g[1]), 2), int(g[2]), int(g[3]),
             round(float(mind.edge_offset(gi)), 3)]
            + [round(float(v), 3) if abs(v) < 100 else None
               for v in mind.carve_edge[gi][:4]]
            + [int(v) for v in mind.carve_edge[gi][4:]]
            for gi, g in enumerate(mind.carve_geo)]  # float(): npz-loaded
        # minds carry np.float32, which json.dumps refuses
    result["events"] = events
    result.update(time_s=round(t, 1), coverage_cells=mind.coverage(),
                  knee_sat=round(sat_knee / max(sat_n, 1), 3),
                  sat_by={k: [round(v[0]/max(v[1],1), 2), round(v[1]*model.opt.timestep, 1)]
                          for k, v in sorted(sat_by.items())},
                  pos=[round(float(data.body("base").xpos[0]), 2),
                       round(float(data.body("base").xpos[1]), 2)])
    if verbose:
        for ev in events:
            print(f"  t={ev[0]:6.1f}  {ev[1]}")
        print(json.dumps({k: v for k, v in result.items() if k != "events"}))
    if video and frames:
        import imageio
        imageio.mimwrite(video, frames, fps=30, quality=8)
        result["video"] = video
    return result

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--dbg", action="store_true")
    ap.add_argument("--video", default=None)
    ap.add_argument("--shortcut", action="store_true")
    ap.add_argument("--corr-obs", action="store_true")
    ap.add_argument("--rocker", action="store_true",
                    help="moving-surface stage: driven rocking deck in the A-corridor")
    ap.add_argument("--hills", action="store_true",
                    help="up-and-over ramps in the B-C corridor (8 deg, 7 cm crest)")
    ap.add_argument("--door", action="store_true",
                    help="hinged door across the chamber entrance")
    ap.add_argument("--survey", action="store_true")
    ap.add_argument("--race", action="store_true")
    ap.add_argument("--mind", default=None, help="mind .npz to load (race) ")
    ap.add_argument("--save-mind", default=None, help="persist the map here at run end")
    ap.add_argument("--max-s", type=float, default=None)
    ap.add_argument("--panic", action="store_true",
                    help="the something-is-chasing-me profile (87.8s maze; knees scream)")
    ap.add_argument("--scenario", choices=["doors"], help="the vault door scenario: 3 runs, tiers shifting, one lock")
    a = ap.parse_args()
    if getattr(a, "scenario", None) == "doors":
        for _title, _wk, _kw in DOOR_SCENARIO:
            print("=" * 72)
            print(_title)
            print("=" * 72)
            _r = run(verbose=True, world_kw=_wk, **_kw)
            print("-> done=%s time=%s reason=%s" % (_r.get("done"),
                                                    _r.get("time_s"),
                                                    _r.get("reason", "-")))
        raise SystemExit(0)
    run(view=a.view, video=a.video, dbg=a.dbg,
        tune=(PANIC if a.panic else None),
        world_kw=({k: v for k, v in dict(shortcut=a.shortcut, corr_obs=a.corr_obs,
                                         rocker=a.rocker,
                                         hills=a.hills,
                                         door=a.door).items() if v} or None),
        race=a.race, mind_path=a.mind, save_mind=a.save_mind, survey=a.survey,
        max_s=a.max_s)
