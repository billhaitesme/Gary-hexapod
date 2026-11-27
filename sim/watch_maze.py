"""Live maze spectator: loops scout-maze runs in a tracking-camera window.
Close the window mid-run to quit after that run. Reimports fresh each lap so
logic edits land between laps. Run: uv run --with mujoco --with numpy python watch_maze.py"""
import importlib, time
lap = 0
while True:
    import gary_mujoco, scout_maze
    importlib.reload(gary_mujoco)     # the WORLD (this was the 'that's not a maze' bug)
    importlib.reload(scout_maze)
    lap += 1
    print(f"[MAZE] lap {lap} — live run on current logic...")
    try:
        r = scout_maze.run(view=True)
    except Exception as e:
        print("[MAZE] viewer closed or error:", e)
        break
    print(f"[MAZE] lap {lap} result: {r}")
    time.sleep(1.0)
