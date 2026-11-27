"""STATIC PROBE: does world_to_leg actually put the foot where I asked?

The wall campaign's last mechanism. The ascent kept falling because
anchor targets were BODY-frame: every body motion dragged planted feet
off their rungs. foot_lock + world_to_leg were built to fix that, but
the transform failed its first outing (feasibility gate rejected its
outputs). Doctrine says: don't guess a convention, run the experiment.

Method (Bill's bring-up plan): stand him up, pick a world point near a
real foot, command it through the transform, step to settle, then
MEASURE where the foot actually went. Error < ~5 mm = the transform is
sound and the ascent can be re-run on true world anchors.

Run: uv run --with "mujoco<3.12" --with numpy python probe_footlock.py
"""
import math
import numpy as np
import mujoco
import gary_mujoco as G


def world_to_leg(model, data, leg, W):
    """world point -> (coxa, out, down) for the driver's leg IK,
    transformed through the LIVE body pose (the door_gym version)."""
    p = data.body("base").xpos
    Rm = data.body("base").xmat.reshape(3, 3)
    b = Rm.T @ (np.array(W) - p)
    mx, my = G.MOUNTS[leg]
    rx, ry = G.RADIAL[leg]
    dx, dy = b[0] - mx, b[1] - my
    out = float(np.hypot(dx, dy))
    down = float(-b[2])
    coxa = float(np.arctan2(dy, dx) - np.arctan2(ry, rx))
    while coxa > np.pi:
        coxa -= 2 * np.pi
    while coxa < -np.pi:
        coxa += 2 * np.pi
    return (coxa, out, down)


def settle(model, data, drv, seconds):
    dt = model.opt.timestep
    n = int(seconds / dt)
    for _ in range(n):
        data.ctrl[:] = drv.ctrl(dt)
        mujoco.mj_step(model, data)


def main():
    model = mujoco.MjModel.from_xml_string(G.build_xml())
    data = mujoco.MjData(model)
    badr = int(model.jnt_qposadr[model.body("base").jntadr[0]])
    data.qpos[badr + 2] = G.COXA_Z
    drv = G.GaitDriver()
    drv.v = drv.vx = drv.omega = 0.0
    drv.lift = 0.0
    settle(model, data, drv, 1.5)

    print("=== STANDING REFERENCE ===")
    base = data.body("base").xpos.copy()
    print("base xyz      : %.4f %.4f %.4f" % tuple(base))
    feet0 = {}
    for i in range(6):
        f = data.geom_xpos[model.geom(f"foot{i}").id].copy()
        feet0[i] = f
        print("foot%d world   : %+.4f %+.4f %+.4f" % (i, f[0], f[1], f[2]))

    # round-trip sanity: transform each foot's OWN position and compare
    # against the neutral leg command the driver would issue.
    print("\n=== ROUND TRIP (foot's own position through the transform) ===")
    for i in range(6):
        coxa, out, down = world_to_leg(model, data, i, feet0[i])
        print("leg%d -> coxa %+.4f rad (%+6.1f deg)  out %.4f  down %.4f"
              % (i, coxa, math.degrees(coxa), out, down))
        ik = G.leg_ik(out, down)
        print("       leg_ik -> %s" % ("FAIL (outside envelope)" if ik is None
                                       else "ok fem %.1f knee %.1f" % ik))

    # THE REAL TEST: command each foot to a point 3 cm forward of where
    # it stands, hold it with foot_lock, and measure the miss.
    print("\n=== COMMAND TEST (target = own position + 3 cm world +y) ===")
    worst = 0.0
    for i in range(6):
        model2 = mujoco.MjModel.from_xml_string(G.build_xml())
        data2 = mujoco.MjData(model2)
        data2.qpos[badr + 2] = G.COXA_Z
        drv2 = G.GaitDriver()
        drv2.v = drv2.vx = drv2.omega = 0.0
        drv2.lift = 0.0
        settle(model2, data2, drv2, 1.5)
        target = data2.geom_xpos[model2.geom(f"foot{i}").id].copy()
        target[1] += 0.03
        cmd = world_to_leg(model2, data2, i, target)
        ik = G.leg_ik(cmd[1], cmd[2])
        if ik is None:
            print("leg%d: command REJECTED by leg_ik (out %.4f down %.4f)"
                  % (i, cmd[1], cmd[2]))
            continue
        drv2.foot_lock = {i: cmd}
        settle(model2, data2, drv2, 1.2)
        got = data2.geom_xpos[model2.geom(f"foot{i}").id].copy()
        err = float(np.linalg.norm(got - target))
        worst = max(worst, err)
        print("leg%d: target %+.4f %+.4f %+.4f -> got %+.4f %+.4f %+.4f "
              "| miss %5.1f mm" % (i, target[0], target[1], target[2],
                                   got[0], got[1], got[2], err * 1000))
    print("\nWORST MISS: %.1f mm  (<5 mm = transform sound)" % (worst * 1000))


if __name__ == "__main__":
    main()
