"""Drive sim-Gary yourself, live, in the MuJoCo viewer window.

Run (keyboard only):
  uv run --with mujoco --with numpy python gary_drive.py
Run (with PS5 DualSense — pair/plug it to the PC first):
  uv run --with mujoco --with numpy --with pygame python gary_drive.py

KEYBOARD (click the viewer window first so it has focus):
  UP / DOWN     forward speed + / -  (10% steps; below 0 = reverse)
  LEFT / RIGHT  turn - / +           (capped 0.45 rad/s; sim yaw reverses above ~0.5 — known issue)
  A / D         strafe left / right  (all gaits are omnidirectional)
  - / =         body height down / up (3 mm) — crouch under the bridge, climb the bars
  G             cycle gait  TRIPOD -> WAVE -> RIPPLE       M   AUTO-gait (Gary picks by demand)
  T             toggle touchdown  match (trained) <-> cosine (old firmware)
  ,  /  .       lift down / up (2 mm)          [  /  ]   cadence slower / faster
  SPACE         full stop            R         reset to start pose
MOUSE: drag = orbit, right-drag = pan, scroll = zoom (viewer built-ins).

PS5 DualSense (same layout as real Gary where it applies):
  Left stick     drive: Y forward/reverse, X strafe — speed ramps with deflection
  Right stick X  turn — tighter with more deflection
  Triangle       cycle gait               Square          toggle touchdown
  Cross          full stop                Circle          reset pose
  L1 / R1        lift down / up           D-pad down/up   cadence slower / faster
  D-pad L / R    body height down / up (crouch / climb posture)
If pygame or the pad is missing, keyboard still works — the pad is optional.

State prints to this console on every change. Close the window or Ctrl+C to quit.
Trained per-gait cadence/lift (2026-08-18, real HPD-0127 torque) are the starting
points, full caps — it's a sim; the firmware carries the derated versions.
"""
import time, math
import numpy as np
import mujoco
from mujoco import viewer as mj_viewer
import gary_mujoco as G

TRAINED = {"TRIPOD": (493, 2.15), "WAVE": (911, 1.09), "RIPPLE": (416, 0.76)}
OMEGA_MAX = 0.45    # rad/s — sim yaw monotonic (~75% effective) below this; reverses above ~0.5 (known issue)

state = dict(gait="TRIPOD", stick=0.0, strafe=0.0, omega=0.0, touchdown="match", auto=False)
model = mujoco.MjModel.from_xml_string(G.build_xml(obstacles=True))   # bars to climb + a bridge to crawl under (+y)
data = mujoco.MjData(model)
drv = None

def make_driver():
    global drv
    cad, lift = TRAINED[state["gait"]]
    d = G.GaitDriver(state["gait"], 1.0, lift_cm=lift, cadence_ms=cad,
                     speed_cap=1.0, touchdown=state["touchdown"])
    if drv is not None:
        d.t, d.lift, d.T = drv.t, d.lift, d.T
    apply_axes(d)
    drv = d

def apply_axes(d=None):
    d = d or drv
    d.v = G.WALK_SPEED_CM_S * 0.01 * state["stick"]
    d.vx = G.WALK_SPEED_CM_S * 0.01 * state["strafe"]
    d.omega = max(-OMEGA_MAX, min(OMEGA_MAX, state["omega"]))

def reset_pose():
    mujoco.mj_resetData(model, data)
    data.qpos[2] = G.COXA_Z

def show():
    print(f"[GARY] {'AUTO:' if state['auto'] else ''}{state['gait']:<6} td={state['touchdown']:<6} "
          f"fwd={state['stick']:+.1f} strafe={state['strafe']:+.1f} turn={drv.omega:+.2f}  "
          f"cadence={drv.T*1000:.0f}ms lift={drv.lift*100:.2f}cm height={drv.height*1000:+.0f}mm", flush=True)

def cycle_gait():
    order = list(TRAINED)
    state["gait"] = order[(order.index(state["gait"]) + 1) % 3]
    make_driver()

def toggle_td():
    state["touchdown"] = "cosine" if state["touchdown"] == "match" else "match"
    make_driver()

def key_cb(keycode):
    k = keycode
    if   k == 265: state["stick"] = min( 1.5, state["stick"] + 0.1)
    elif k == 264: state["stick"] = max(-1.0, state["stick"] - 0.1)
    elif k == 262: state["omega"] = min( OMEGA_MAX, state["omega"] + 0.1)
    elif k == 263: state["omega"] = max(-OMEGA_MAX, state["omega"] - 0.1)
    elif k in (ord('G'), ord('g')): cycle_gait()
    elif k in (ord('T'), ord('t')): toggle_td()
    elif k == ord(','): drv.lift = max(0.002, drv.lift - 0.002)
    elif k == ord('.'): drv.lift = min(0.035, drv.lift + 0.002)
    elif k == ord('['): drv.T = min(3.0, drv.T * 1.1)
    elif k == ord(']'): drv.T = max(0.2, drv.T / 1.1)
    elif k in (ord('A'), ord('a')): state["strafe"] = max(-1.0, state["strafe"] - 0.1)
    elif k in (ord('D'), ord('d')): state["strafe"] = min( 1.0, state["strafe"] + 0.1)
    elif k == ord('-'): drv.height = max(-0.022, drv.height - 0.003)
    elif k == ord('='): drv.height = min( 0.045, drv.height + 0.003)
    elif k in (ord('M'), ord('m')): state["auto"] = not state["auto"]
    elif k == 32: state["stick"] = 0.0; state["strafe"] = 0.0; state["omega"] = 0.0
    elif k in (ord('R'), ord('r')): reset_pose()
    else: return
    apply_axes(); show()

# ---- optional PS5 DualSense via pygame --------------------------------------
pad = None
try:
    import pygame
    pygame.init(); pygame.joystick.init()
    if pygame.joystick.get_count() > 0:
        pad = pygame.joystick.Joystick(0); pad.init()
        print(f"[PAD] using controller: {pad.get_name()}")
    else:
        print("[PAD] no controller found — keyboard only.")
except Exception as e:
    print(f"[PAD] pygame unavailable ({e}) — keyboard only.")

_pad_prev = {}
def pad_poll():
    """Sticks = speed/turn; buttons edge-triggered. SDL DualSense mapping:
    axes 0/1 LS, 2/3 RS; buttons 0 cross 1 circle 2 square 3 triangle 9 L1 10 R1; hat 0 dpad."""
    pygame.event.pump()
    dead = 0.12
    ls = -pad.get_axis(1); lx = pad.get_axis(0); rs = pad.get_axis(2)
    ls = 0.0 if abs(ls) < dead else ls
    lx = 0.0 if abs(lx) < dead else lx
    rs = 0.0 if abs(rs) < dead else rs
    changed = False
    new_stick, new_strafe = round(ls, 2), round(lx, 2)
    new_om = round(rs * OMEGA_MAX, 2)
    if (abs(new_stick - state["stick"]) > 0.04 or abs(new_strafe - state["strafe"]) > 0.04
            or abs(new_om - state["omega"]) > 0.04):
        state["stick"], state["strafe"], state["omega"] = new_stick, new_strafe, new_om
        apply_axes(); changed = True
    def edge(idx, name):
        cur = pad.get_button(idx); was = _pad_prev.get(name, 0)
        _pad_prev[name] = cur
        return cur and not was
    if edge(3, "tri"):   cycle_gait(); changed = True
    if edge(2, "sq"):    toggle_td();  changed = True
    if edge(0, "cross"):
        state["stick"] = 0.0; state["strafe"] = 0.0; state["omega"] = 0.0; apply_axes(); changed = True
    if edge(1, "circ"):  reset_pose(); changed = True
    if edge(9, "l1"):    drv.lift = max(0.002, drv.lift - 0.002); changed = True
    if edge(10, "r1"):   drv.lift = min(0.035, drv.lift + 0.002); changed = True
    hat = pad.get_hat(0) if pad.get_numhats() else (0, 0)
    if hat[1] and not _pad_prev.get("haty", 0):
        drv.T = drv.T * 1.1 if hat[1] < 0 else drv.T / 1.1
        drv.T = min(3.0, max(0.2, drv.T)); changed = True
    if hat[0] and not _pad_prev.get("hatx", 0):
        drv.height = max(-0.022, min(0.045, drv.height + (0.003 if hat[0] > 0 else -0.003)))
        changed = True
    _pad_prev["haty"], _pad_prev["hatx"] = hat[1], hat[0]
    if changed: show()

# ---- main loop ---------------------------------------------------------------
make_driver()
reset_pose()
print(__doc__)
show()
def auto_gait_tick():
    # The seed of "Gary decides": pick gait from commanded demand, with hysteresis.
    # Slow/precise -> WAVE (5 feet down), medium -> RIPPLE, fast -> TRIPOD.
    demand = max(abs(state["stick"]), abs(state["strafe"]), abs(state["omega"]) / OMEGA_MAX * 0.8)
    want = "WAVE" if demand < 0.32 else ("RIPPLE" if demand < 0.68 else "TRIPOD")
    if want != state["gait"]:
        state["gait"] = want
        make_driver()
        show()

with mj_viewer.launch_passive(model, data, key_callback=key_cb) as v:
    v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING     # camera follows Gary
    v.cam.trackbodyid = model.body("base").id
    v.cam.distance, v.cam.elevation, v.cam.azimuth = 0.7, -20, 140
    wall0 = time.perf_counter(); t = 0.0; n = 0
    while v.is_running():
        if pad is not None and n % 25 == 0:   # poll pad at ~20 Hz
            pad_poll()
        if state["auto"] and n > 0 and n % 1000 == 0:   # re-evaluate every 2 s
            auto_gait_tick()
        data.ctrl[:] = drv.ctrl(model.opt.timestep)
        mujoco.mj_step(model, data)
        t += model.opt.timestep; n += 1
        v.sync()
        lag = (wall0 + t) - time.perf_counter()
        if lag > 0: time.sleep(lag)
print("viewer closed.")
