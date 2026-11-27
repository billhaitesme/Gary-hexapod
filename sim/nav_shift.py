"""CURRICULUM FINAL EXAM — THE SHIFTING LABYRINTH (Bill, 2026-08-20: walls
change so his map goes stale: BELIEF REVISION — "fresh sensing overrides
memory; replan on contradiction. Justifying his decision making.")

He races the shortcut maze he has fully surveyed. When he's committed (through
the C doorway, heading for the chamber), the world CHANGES BEHIND HIS BELIEFS:
the D-gap (chamber entry, x 1.55-2.0) slides east (x 1.85-2.30). His map now
lies. What should happen, and what this script verifies:
  1. he walks to where the door WAS -> ring says wall -> evidence overwrites
     memory (see-through decay + endpoint hits),
  2. replanning against the revised map fails everywhere -> IMPASSE ->
     mind.doubt(): wall evidence halves, unverified walls demote to UNKNOWN,
  3. the frontier machinery re-explores, finds the moved gap, and he finishes.

Run:  uv run --with mujoco --with numpy python nav_shift.py         (headless)
      ... --view                                                    (watch)
      ... --explore   (no prior mind: shift fires mid-exploration instead)
"""
import argparse, json
import nav_maze as NV


def dgap_slide(model):
    d1 = model.geom("mzD1").id
    d2 = model.geom("mzD2").id
    model.geom_pos[d1][0] = 1.475      # D1 grows to cover x 1.10..1.85
    model.geom_size[d1][0] = 0.375
    model.geom_pos[d2][2] = -0.5       # D2 sinks away: new gap = x 1.85..2.30


SHIFTS = [dict(name="chamber door slides east (his map now lies)",
               when=lambda t, x, y: x > 1.15 and y < 0.90,
               apply=dgap_slide)]


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--dbg", action="store_true")
    ap.add_argument("--explore", action="store_true",
                    help="take the exam without the surveyed mind")
    ap.add_argument("--video", default=None)
    a = ap.parse_args()
    r = NV.run(view=a.view, dbg=a.dbg, video=a.video,
               world_kw=dict(shortcut=True),
               race=not a.explore,
               mind_path=(None if a.explore else "out/mind_shortcut.npz"),
               shifts=SHIFTS)
    print("EXAM", "PASSED" if r.get("done") else "FAILED", json.dumps(r))
