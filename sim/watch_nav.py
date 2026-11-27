"""Live NAVIGATOR spectator (Bill: "integrate live map with the sim" — watch
the world and his MIND side by side): TWO persistent windows — a rendered
tracking-camera view of the world (the reel camera, live, real-time paced)
and the occupancy grid in Gary's head with his position, heading, planned
path, goal, gimbal standpoints, measured carve lines, and wall glances.

The SAME two windows update lap after lap — nothing closes and reopens
(Bill: "it kept opening new windows"). Close the WORLD window to quit
immediately; close the MAP window to quit when the current lap ends.
Reimports logic fresh each lap, so my edits land between laps while you
keep watching.

Run:  uv run --with "mujoco<3.12" --with numpy --with matplotlib python watch_nav.py
      ... --panic          (the record profile)
      ... --race           (race the known map)
      ... --shortcut       (shortcut + tipping-plate world)
      ... --rocker         (moving-surface stage: the rocking deck)
"""
import argparse, importlib, math, time
import numpy as np


class SpectatorClosed(Exception):
    pass


def make_world_window(cam):
    import matplotlib.pyplot as plt
    plt.ion()
    fig, ax = plt.subplots(num="Gary — world  (drag to orbit, scroll to zoom)",
                           figsize=(7.4, 4.8))
    im = ax.imshow(np.zeros((400, 640, 3), dtype=np.uint8))
    ax.set_axis_off()
    fig.tight_layout(pad=0.2)
    # free camera (Bill: "free the camera so i can hover around Gary") —
    # drag orbits, wheel zooms; the camera keeps TRACKING him while you orbit
    drag = {"xy": None}

    def on_press(e):
        drag["xy"] = (e.x, e.y)

    def on_release(e):
        drag["xy"] = None

    def on_move(e):
        if drag["xy"] is None:
            return
        dx, dy = e.x - drag["xy"][0], e.y - drag["xy"][1]
        drag["xy"] = (e.x, e.y)
        cam["azimuth"] = (cam["azimuth"] - 0.5 * dx) % 360
        cam["elevation"] = max(-89.0, min(-5.0, cam["elevation"] + 0.4 * dy))

    def on_scroll(e):
        cam["distance"] = max(0.25, min(3.5, cam["distance"]
                                        * (0.88 if e.button == "up" else 1.14)))

    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("button_release_event", on_release)
    fig.canvas.mpl_connect("motion_notify_event", on_move)
    fig.canvas.mpl_connect("scroll_event", on_scroll)
    fig.show()
    return dict(fig=fig, im=im)


def make_cloud_window():
    import matplotlib.pyplot as plt
    plt.ion()
    fig = plt.figure(num="Gary — scan cloud (what his scans painted)",
                     figsize=(6.6, 5.2))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim(-0.4, 2.6); ax.set_ylim(-0.3, 1.9); ax.set_zlim(0, 0.42)
    ax.set_box_aspect((3.0, 2.2, 0.55))
    sc = ax.scatter([], [], [], s=2, c=[], cmap="plasma", vmin=0.0, vmax=0.32)
    ax.set_title("3D ray endpoints, scans only — sparse on purpose", fontsize=9)
    fig.tight_layout()
    fig.show()
    return dict(fig=fig, sc=sc, pts=[])


def draw_cloud(C, new_pts):
    import matplotlib.pyplot as plt
    if not new_pts or not plt.fignum_exists(C["fig"].number):
        return
    C["pts"].extend(new_pts)
    P = np.array(C["pts"][-15000:])
    C["sc"]._offsets3d = (P[:, 0], P[:, 1], P[:, 2])
    C["sc"].set_array(P[:, 2])
    C["fig"].canvas.draw_idle()
    C["fig"].canvas.flush_events()


def show_frame(WW, fr):
    import matplotlib.pyplot as plt
    if not plt.fignum_exists(WW["fig"].number):
        raise SpectatorClosed()      # world window closed = quit NOW
    WW["im"].set_data(fr)
    WW["fig"].canvas.draw_idle()
    WW["fig"].canvas.flush_events()


def make_map_window(NV):
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    plt.ion()
    fig, ax = plt.subplots(num="Gary's mind — onboard map", figsize=(7.6, 5.9))
    # UNK charcoal / FREE bone / WALL near-black / LOW amber (the crouch lanes)
    cmap = ListedColormap(["#33333c", "#e9e5da", "#101014", "#e8912d"])
    im = ax.imshow(np.zeros((NV.GNY, NV.GNX)), origin="lower", cmap=cmap,
                   vmin=0, vmax=3, interpolation="nearest",
                   extent=(NV.GX0, NV.GX0 + NV.GNX*NV.CS,
                           NV.GY0, NV.GY0 + NV.GNY*NV.CS))
    path_ln, = ax.plot([], [], "-", color="#3fa34d", lw=2.5, label="planned path")
    goal_mk, = ax.plot([], [], "*", color="#d43d3d", ms=15, label="goal")
    scan_mk, = ax.plot([], [], "x", color="#9b59d0", ms=9, mew=2.5, label="gimbal scans")
    carve_ln, = ax.plot([], [], "--", color="#f5c542", lw=2.2,
                        label="measured lane line")
    glance_mk, = ax.plot([], [], "x", color="#ff2222", ms=7, mew=2,
                         label="wall glances")
    bot_mk, = ax.plot([], [], "o", color="#2d7fe8", ms=9, label="Gary")
    nose_ln, = ax.plot([], [], "-", color="#2d7fe8", lw=2)
    ax.axvline(NV.FINISH_MAST_X, color="#3fa34d", ls="--", lw=1, alpha=0.6)
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=8, framealpha=0.9)
    fig.tight_layout()
    fig.show()
    return dict(fig=fig, ax=ax, im=im, path=path_ln, goal=goal_mk,
                scan=scan_mk, carve=carve_ln, glance=glance_mk,
                bot=bot_mk, nose=nose_ln, nv=NV)


def draw(W, s):
    import matplotlib.pyplot as plt
    if not plt.fignum_exists(W["fig"].number):
        return          # map window closed mid-run — keep the sim going
    NV = W["nv"]
    W["im"].set_data(s["grid"].T)          # grid is [x][y]; imshow rows are y
    if s["path"]:
        xs, ys = zip(*(NV.world_of(c) for c in s["path"]))
        W["path"].set_data(xs, ys)
    else:
        W["path"].set_data([], [])
    if s["goal"] is not None:
        gx, gy = NV.world_of(s["goal"])
        W["goal"].set_data([gx], [gy])
    else:
        W["goal"].set_data([], [])
    if s["scans"]:
        sx, sy = zip(*s["scans"])
        W["scan"].set_data(sx, sy)
    cx, cy = [], []
    for ax_, ay_, dx_, dy_, off in s.get("carves", []):
        # the measured line: anchor + axis, shifted by the edge-bracket
        # offset — watching this snap onto the true centerline after a
        # micro-peek is the whole mouth machinery in one glance
        for d in (-0.15, 0.90):
            cx.append(ax_ + dx_*d - dy_*off)
            cy.append(ay_ + dy_*d + dx_*off)
        cx.append(float("nan")); cy.append(float("nan"))
    W["carve"].set_data(cx, cy)
    if s.get("glances"):
        gx, gy = zip(*s["glances"])
        W["glance"].set_data(gx, gy)
    else:
        W["glance"].set_data([], [])
    W["bot"].set_data([s["x"]], [s["y"]])
    W["nose"].set_data([s["x"], s["x"] - 0.10*math.sin(s["yaw"])],
                       [s["y"], s["y"] + 0.10*math.cos(s["yaw"])])
    W["ax"].set_title(f"t={s['t']:6.1f}s   {s['mode']}   "
                      f"known {s['coverage']} cells   "
                      f"glances {s.get('wall_hits', 0)}", fontsize=10)
    W["fig"].canvas.draw_idle()
    W["fig"].canvas.flush_events()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--shortcut", action="store_true")
    ap.add_argument("--race", action="store_true")
    ap.add_argument("--panic", action="store_true",
                    help="watch the PANIC record profile live")
    ap.add_argument("--rocker", action="store_true",
                    help="moving-surface stage: rocking deck in the A-corridor")
    ap.add_argument("--hills", action="store_true",
                    help="up-and-over ramps in the B-C corridor")
    ap.add_argument("--door", action="store_true",
                    help="hinged door across the chamber entrance")
    ap.add_argument("--mind", default=None,
                    help="race a specific saved mind (default: the mode's usual)")
    ap.add_argument("--scenario", choices=["doors"],
                    help="the vault door scenario: 3 runs, tiers shifting, one lock")
    a = ap.parse_args()
    lap = 0
    W = WW = C = None
    CAM = dict(azimuth=140.0, elevation=-50.0, distance=1.1)
    while True:
        import gary_mujoco, door_gym, nav_maze
        importlib.reload(gary_mujoco)      # the WORLD too (stale-window lesson)
        importlib.reload(door_gym)         # BreachFSM lives here -- nav
        importlib.reload(nav_maze)         # reload alone left it stale
        lap += 1
        print(f"[NAV] lap {lap} — live run on current logic...")
        if W is None:
            W = make_map_window(nav_maze)
            WW = make_world_window(CAM)
            C = make_cloud_window()
        else:
            W["nv"] = nav_maze             # fresh module after the reload
            C["pts"].clear()               # new lap paints a fresh cloud
        wk = {k: True for k, v in dict(shortcut=a.shortcut, rocker=a.rocker,
                                       hills=a.hills, door=a.door).items() if v} or None
        try:
            if a.scenario == "doors":
                for _title, _wk, _kw in nav_maze.DOOR_SCENARIO:
                    print("[NAV] " + _title)
                    W["fig"].canvas.manager.set_window_title(
                        "Gary's mind -- " + _title.split(" -- ")[0])
                    _r = nav_maze.run(verbose=False,
                                      map_cb=lambda s: (draw(W, s),
                                                        draw_cloud(C, s.get("scan3d", []))),
                                      frame_cb=lambda fr: show_frame(WW, fr),
                                      cam_ctl=CAM, world_kw=_wk, **_kw)
                    _r.pop("events", None)
                    print("[NAV] result:", _r)
                continue
            r = nav_maze.run(verbose=False,
                             map_cb=lambda s: (draw(W, s),
                                               draw_cloud(C, s.get("scan3d", []))),
                             frame_cb=lambda fr: show_frame(WW, fr),
                             cam_ctl=CAM,
                             world_kw=wk, race=a.race,
                             tune=(nav_maze.PANIC if a.panic else None),
                             mind_path=(a.mind if a.mind else
                                        ("out/mind_shortcut.npz" if a.shortcut
                                         else "out/mind_door.npz" if a.door
                                         else "out/mind_base.npz") if a.race
                                        else None))
        except SpectatorClosed:
            print("[NAV] world window closed — done spectating.")
            break
        except Exception as e:
            print("[NAV] error:", e)
            break
        r.pop("events", None)
        print(f"[NAV] lap {lap} result: {r}")
        import matplotlib.pyplot as plt
        if not plt.fignum_exists(W["fig"].number):
            print("[NAV] map window closed — done spectating.")
            break
        W["fig"].canvas.manager.set_window_title(
            f"Gary's mind — lap {lap}: {'SOLVED in %.0fs' % r['time_s'] if r.get('done') else r.get('reason','')}")
        time.sleep(2.0)
