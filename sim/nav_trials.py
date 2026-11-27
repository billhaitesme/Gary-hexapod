"""CURRICULUM STAGE 1 — TIME TRIALS (Bill: "reward fast, punish slow").

Evolves the navigator's drive/planner genes (nav_maze.TUNE) against the maze
clock: fitness = finish time, DNF = 1200 + 200 * distance-short, fall = worse.
(1+lambda) evolution with parallel workers; the sim is deterministic so each
gene set is exactly one run. Baseline: hand-tuned genes solve in 407.6 s.

Run:  uv run --with mujoco --with numpy python nav_trials.py
      ... --gens 6 --lam 6
Best genes land in out/nav_trials.json; apply by pasting into nav_maze.TUNE.
"""
import argparse, json, os, random
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))

# OVERDRIVE (Bill): cruise may exceed nominal (the trained tripod ran ~1.5x
# at 6.4% knee sat) and cadence is a gene — but speed above nominal is PRICED
# by knee saturation in the fitness (this project began with burned knees).
BOUNDS = dict(cruise=(0.60, 2.50), duck=(0.40, 1.00), replan=(1.0, 4.0),
              carrot=(2, 6), kyaw=(1.0, 2.5), fwd_full=(0.20, 0.60),
              yaw_lead=(0.0, 0.65),   # cornering yaw lead (lo=legacy off;
              # hand-set 0.45 broke both records: race 108.6->98.2,
              # panic 71.1->68.0 -- Bill: 'let it run its generations')
              fwd_stop=(0.60, 1.40), cadence=(420, 700), lift=(0.008, 0.030), crab=(0.0, 1.2))
# race mode also evolves risk genes: care under tilt, and appetite for LOW
# (crouch/shortcut) lanes — the tipping plate prices them
RACE_BOUNDS = dict(BOUNDS, tilt_slow=(0.25, 1.00), low_extra=(0.0, 8.0))
# PANIC MODE (Bill: "how fast can it actually go if it NEEDS to") — no knee
# pricing, pure clock, falls still fatal. The clutch protects the gears; if
# survival demands it, the clutch can scream.
PANIC_BOUNDS = dict(BOUNDS, cruise=(1.5, 3.6), zoom=(1.5, 3.6), duck=(0.4, 2.8),
                    brake_at=(0.12, 0.50), brake_slope=(0.0, 3.0))
# duck ceiling raised: the crawl-strip probe showed the crouched clutch slips
# ~40% — commanding HARDER pushes more through the slip (cmd 9 -> 5 cm/s
# actual) — but only evolution can integrate a hot crawl into a whole run
RACE = False           # set by main(); workers re-derive it from the task


def clamp(g, bounds=None):
    out = {}
    for k, (lo, hi) in (bounds or BOUNDS).items():
        # g.get(k, lo): a stored best from before a gene existed enters at
        # the LOW bound — new genes are built so lo = legacy behavior
        v = min(max(g.get(k, lo), lo), hi)
        out[k] = int(round(v)) if k in ("carrot", "cadence") else round(float(v), 3)
    return out


def mutate(g, sigma, rng, bounds):
    child = dict(g)
    for k, (lo, hi) in bounds.items():
        if rng.random() < 0.6:
            child[k] = g[k] + rng.gauss(0, sigma * (hi - lo))
    return clamp(child, bounds)


def evaluate(task):
    genes, race = task
    panic = race == "panic"
    import nav_maze
    if panic:
        r = nav_maze.run(verbose=False, tune=genes)
        if r["done"]:
            # clock + WALL GLANCES priced (Bill: "glances are friction" —
            # and unpriced, evolution sold cornering margin for clock time
            # every cycle until the tactical diameter outgrew the corridors:
            # brake_slope decayed 2.03 -> 0.63 across one evening)
            return r["time_s"] + 2.0 * r.get("wall_hits", 0), r
        return (2000.0 if r["reason"] == "fell"
                else 1200.0 + 200.0 * max(0.0, nav_maze.FINISH_X - r["pos"][0])), r
    if race:
        r = nav_maze.run(verbose=False, tune=genes, race=True,
                         world_kw=dict(shortcut=True),
                         mind_path=os.path.join(HERE, "out", "mind_shortcut.npz"))
    else:
        r = nav_maze.run(verbose=False, tune=genes)
    if r["done"]:
        # knee saturation above a 12% budget is priced hard: overdrive must
        # be earned from the clutch, not stolen from the gearboxes
        score = (r["time_s"] + 800.0 * max(0.0, r.get("knee_sat", 0.0) - 0.12)
                 + 2.0 * r.get("wall_hits", 0))
    elif r["reason"] == "fell":
        score = 2000.0
    else:
        score = 1200.0 + 200.0 * max(0.0, nav_maze.FINISH_X - r["pos"][0])
    return score, r


def main(gens, lam, seed, race=False):
    rng = random.Random(seed)
    import nav_maze
    bounds = (PANIC_BOUNDS if race == "panic" else
              RACE_BOUNDS if race else BOUNDS)
    save = os.path.join(HERE, "out",
                        "nav_panic.json" if race == "panic" else
                        "nav_race.json" if race else "nav_trials.json")
    if os.path.exists(save):
        prev = json.load(open(save))
        best, best_score = clamp(prev["best"], bounds), prev["score"]
        hist = prev["history"]
        print(f"resuming: stored score {best_score:.1f}  {best}", flush=True)
        # ALWAYS re-evaluate the incumbent: the stored score belongs to the
        # mechanics it ran on. A behavior fix landed between sessions once and
        # mutants spent 4 generations losing to a ghost — genuinely better
        # kids (85.8 vs a real 86.1) were rejected against a stale 85.7.
        best_score, r = evaluate((best, race))
        print(f"  incumbent on current mechanics: {best_score:.1f}", flush=True)
    else:
        best = clamp(dict(nav_maze.TUNE), bounds)
        print("evaluating baseline genes...", flush=True)
        best_score, r = evaluate((best, race))
        print(f"  baseline: score {best_score:.1f}  {r['reason']}  pos {r['pos']}", flush=True)
        hist = [dict(gen=0, score=best_score, genes=best)]
    with ProcessPoolExecutor(max_workers=lam) as pool:
        for gen in range(1, gens + 1):
            sigma = 0.22 * (0.82 ** (gen - 1))
            kids = [mutate(best, sigma, rng, bounds) for _ in range(lam)]
            results = list(pool.map(evaluate, [(k, race) for k in kids]))
            for k, (s, r) in zip(kids, results):
                tag = (f"{r['time_s']:.0f}s sat={r.get('knee_sat', 0):.2f}"
                       if r["done"] else r["reason"])
                print(f"  gen {gen}  score {s:7.1f} ({tag})  {k}", flush=True)
                if s < best_score:
                    best_score, best = s, k
            hist.append(dict(gen=gen, score=best_score, genes=best))
            print(f"gen {gen} BEST score {best_score:.1f}  {best}", flush=True)
            with open(save, "w") as f:
                json.dump(dict(best=best, score=best_score, history=hist), f, indent=1)
    print(json.dumps(dict(best=best, score=best_score)))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--gens", type=int, default=6)
    ap.add_argument("--lam", type=int, default=6)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--race", action="store_true",
                    help="race the KNOWN shortcut maze (needs out/mind_shortcut.npz)")
    ap.add_argument("--panic", action="store_true",
                    help="pure-clock fitness, no knee pricing: how fast if he NEEDS to")
    a = ap.parse_args()
    main(a.gens, a.lam, a.seed, race=("panic" if a.panic else a.race))
