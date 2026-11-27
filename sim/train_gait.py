"""Stage-3: optimize Gary's gait parameters inside the MuJoCo twin (CMA-ES).

Optimizes, per gait, the three knobs that map 1:1 onto firmware constants:
    cadence_ms  -> GAIT_MIN_CADENCE[]
    lift_cm     -> CUSTOM_GAIT_LIFT_CM
    speed_cap   -> GAIT_SPEED_CAP[]
Objective: walk fast WITHOUT abusing the knees:
    score = achieved_v  -  0.05 * knee_sat_pct  -  50 * fell_over
The knee-saturation term is the whole point — it is the sim proxy for the stall
current that killed the real knee servos.

Run:  uv run --with mujoco --with numpy --with cma python train_gait.py --gait TRIPOD --gens 10
Writes out/train_<gait>.json with the best parameters + before/after metrics.
"""
import argparse, json, os, math
import numpy as np
import cma
import gary_mujoco as G

HERE = os.path.dirname(os.path.abspath(__file__))

# search space: x is normalized [0,1]^3
BOUNDS = dict(cadence_ms=(300, 2000), lift_cm=(0.6, 3.0), speed_cap=(0.05, 1.0))
KEYS = list(BOUNDS)

def decode(x):
    p = {}
    for v, k in zip(x, KEYS):
        lo, hi = BOUNDS[k]
        p[k] = lo + float(np.clip(v, 0, 1)) * (hi - lo)
    p["cadence_ms"] = int(p["cadence_ms"])
    return p

def score(gait, p, seconds=5.0):
    r = G.run(gait, 1.0, seconds, lift_cm=p["lift_cm"], cadence_ms=p["cadence_ms"],
              speed_cap=p["speed_cap"], touchdown="match")
    fell = r["body_height_end"] < 0.03
    s = r["achieved_v_cm_s"] - 0.05 * r["knee_sat_pct"] - (50 if fell else 0)
    return s, r

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gait", default="TRIPOD", choices=G.GAITS)
    ap.add_argument("--gens", type=int, default=10)
    ap.add_argument("--pop", type=int, default=8)
    a = ap.parse_args()

    baseline_p = dict(cadence_ms=G.MIN_CADENCE_MS[G.GAITS.index(a.gait)], lift_cm=2.5,
                      speed_cap=G.SPEED_CAP[G.GAITS.index(a.gait)])
    b_s, b_r = score(a.gait, baseline_p)
    print("BASELINE", json.dumps(b_r), "score", round(b_s, 2), flush=True)

    es = cma.CMAEvolutionStrategy([0.5, 0.5, 0.5], 0.25,
                                  {"popsize": a.pop, "bounds": [0, 1], "seed": 7, "verbose": -9})
    best = (b_s, baseline_p, b_r)
    for gen in range(a.gens):
        xs = es.ask()
        losses = []
        for x in xs:
            p = decode(x)
            s, r = score(a.gait, p)
            losses.append(-s)
            if s > best[0]:
                best = (s, p, r)
        es.tell(xs, losses)
        print(f"gen {gen+1}/{a.gens}  best_score={round(best[0],2)}  best={best[1]}", flush=True)

    out = dict(gait=a.gait, baseline=dict(params=baseline_p, metrics=b_r, score=round(b_s, 2)),
               best=dict(params=best[1], metrics=best[2], score=round(best[0], 2)))
    os.makedirs(os.path.join(HERE, "out"), exist_ok=True)
    path = os.path.join(HERE, "out", f"train_{a.gait.lower()}.json")
    json.dump(out, open(path, "w"), indent=1)
    print("WROTE", path)
    print(json.dumps(out, indent=1))

if __name__ == "__main__":
    main()
