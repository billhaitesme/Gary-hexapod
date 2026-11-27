# The navigator book broke on rubber feet — bisected 2026-08-22

While re-verifying the records after the knee correction, the shipped PANIC
profile (`nav_maze.py --panic`, genes v12 "CLEAN CLASS 83.2 s / zero contact")
returned **123.3 s with 138 wall hits** on the commit before the knee work.
Not chaos: 1e-6 gene nudges move it by +-1 s. A worktree bisect of the 49
commits touching nav/physics since the book (`b0ab0d0`):

| commit | what | PANIC |
|---|---|---|
| `b0ab0d0` | the book | **75.1 s, 0 hits** (reproduces exactly) |
| `29f8d45` | DOOR world | 75.1 s, 0 hits |
| `5621236` | wall-climb gait gen 1 | 75.1 s, 0 hits |
| `d92bca6` | **rubber-foot calibration: friction 1.2 -> 2.0 on all feet** | **115.1 s, 141 hits** |
| `3c46ce0` | ratchet genome | 115.1 s, 141 hits |
| `6c91ca4` … `cc13e7f` … `54ff148` | everything after | 115-123 s, 138-141 hits |

So the break is exact: the rubber caps (Bill: "I was thinking his feet had
more traction than that -- they do in reality"; the commit itself says
MEASURE IN SEPT) were applied globally and the navigator's genes -- evolved
at 1.2 -- were never re-run under them. The book was written at friction 1.2.
Every nav record quoted since 2026-08-22 morning has been a friction-1.2
number running on a friction-2.0 Gary.

What it is not: the meshes, the render commit, the ladder worlds and the knee
correction. `GARY_FOOT_MU=1.2` on HEAD is the closing check (result appended
below when it lands).

Decision owed to Bill: the true friction of his rubber caps on his floor is a
September measurement; 2.0 and 1.2 are both guesses. Until then either (a)
re-evolve the navigator (nav_trials PANIC/race) at 2.0 and re-baseline the
book honestly, or (b) hold the book at 1.2 with this caveat and do the
re-tune once the real number exists. The navigator is demonstrably
friction-sensitive (40 s, 141 hits), so whichever number September gives,
the nav genes must be re-tuned for it before the hardware runs them.

## Closing check — and what it did NOT close

`GARY_FOOT_MU=1.2` on HEAD (free knee + IK clamp + meshes + render + every
nav commit since the book): **PANIC 129.0 s, 205 wall hits.** Not 75.1 / 0.
So rubber feet are the first break (exact, 0 -> 141 hits at `d92bca6`) but
not the only one: the nav-logic commits after it (door curiosity, walk-through
verdicts, BreachFSM port, yaw-lead, untangle reflex, resweep fix …) changed
the navigator's behaviour as well, and the book was never re-run as a whole
after any of them. **The nav book is not reproducible at HEAD under either
friction.**

Second lesson, from the knee runs on pre-knee physics: free joint = 123.3 s
and the original joint stop = 123.3 s (bit-identical), but the IK clamp at
the same 101.7 deg = 81.1 s with a different CROUCH PEEK verdict ("seeing
1.55 m past the wall" vs 1.50). The knee command exceeds 101.7 deg only
10 times in 75,000 IK calls, yet those happen in crouches, and the peek
verdict is an exact threshold: sub-degree crouch geometry flips a discrete
navigation decision worth 40 s, while a 1e-6 gene nudge moves the clock
+-1 s. A single PANIC time is therefore NOT a robust statistic. The commit
message of 4de3e56 ("walking is unchanged by the knee") is right about the
kinematics and wrong about the outcome; this paragraph is the correction.

What the book needs before it is quoted again: a RECORD HARNESS -- N runs
per configuration with small perturbations (the nudge protocol above),
report median time and wall-hit count, pass/fail on both -- and a
re-baseline under the friction Bill chooses. The navigator's logic has
moved on from the book's build; its numbers have to move with it,
measured, or be retired.
