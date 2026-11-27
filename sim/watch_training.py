"""Spectator: replay the best-known course policy in a live window, on repeat.
Reloads out/course_policy_sensor.json before every lap, so while training runs
(it checkpoints each generation) you literally watch Gary get better.
Close the window during a lap to quit after that lap. Mouse: drag orbit, scroll zoom.
Run: uv run --with mujoco --with numpy python watch_training.py
"""
import json, time, os
import train_course_sensor as T

LAP = 0
while True:
    try:
        d = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        "out", "course_policy_sensor.json")))
    except Exception as e:
        print("no policy yet:", e); time.sleep(5); continue
    LAP += 1
    gen = d.get("gen", "final")
    print(f"[WATCH] lap {LAP} — replaying best (gen {gen}) "
          f"prog={d.get('metrics',{}).get('progress_y','?')}m ...", flush=True)
    try:
        sc, r = T.rollout(d["params"], view=True, verbose=True)
    except Exception as e:
        print("[WATCH] viewer closed or error:", e)
        break
    time.sleep(1.0)
