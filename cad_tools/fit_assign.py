"""EM fit of the vendor model's POSED joint angles + part assignment.

The first mesh exports trusted leg_geometry.json's foot_tip points;
they are ~69 mm from the posed feet, and every heuristic built on them
mis-filed parts (Bill, watching the spectator live: "those parts did
NOT go back together the way you took them apart"). This stops
assuming:

  E-step: each object votes for the nearest link capsule (or one of
          two TIGHT chassis boxes -- a generous box swallows the robot,
          the first fit starved four legs to zero) by 25th-percentile
          sampled-vertex distance
  M-step: each leg's (coxa, femur, knee) angles refit by grid+refine
          to minimize its parts' distances to the FK link segments

with an azimuth-lobe init round so every chain finds its own limb
before the chassis competes. FK constants import from gary_mujoco --
no transcription. Result (2026-08-21): all legs ~zero pose (the sim's
FEM0/KNEE0 came from this file) except rear coxas swung +/-10 deg;
22-26 parts per leg, mirror-symmetric, mean axis distance ~12-13 mm
(half a servo width = healthy).

Run: uv run --with "mujoco<3.12" --with numpy python fit_assign.py
Writes out/fit_result.json (pose_deg, labels, resid_mm).
"""
import json, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "sim"))
import gary_mujoco as G

SP = os.path.join(HERE, "out")
L1, L2, L3 = G.L1, G.L2, G.L3
FEM_DROP, FEM0, KNEE0 = G.FEM_DROP, G.FEM0, G.KNEE0
MOUNTS, RADIAL = G.MOUNTS, G.RADIAL
Y_OFF, Z_OFF = 6.30, 58.42

z = np.load(os.path.join(SP, "cad_objects.npz"))
V, voff, layer = z["V"], z["voff"], z["layer"]
NOBJ = len(voff) - 1
print("objects:", NOBJ)

samples = []
for o in range(NOBJ):
    P = V[voff[o]:voff[o + 1]].astype(np.float64)
    # 180-deg yaw: the hex mount ring is centro-symmetric, so hubs
    # match the sim mounts to 0.03 mm BOTH ways -- the sonar face
    # broke the tie (Bill: 'the red dot, and subsequently the LiDAR
    # are on the wrong side'). Sim forward = +y = his face.
    P = np.column_stack([-P[:, 0], -(P[:, 1] + Y_OFF), P[:, 2] - Z_OFF]) / 1000.0
    step = max(1, len(P) // 40)
    samples.append(P[::step][:40])
sizes = np.array([len(s) for s in samples])


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def chain(i, qc, qf, qk):
    mx, my = MOUNTS[i]
    rx, ry = RADIAL[i]
    yaw0 = np.arctan2(ry, rx)
    Rc = Rz(yaw0 + qc)
    tc = np.array([mx, my, 0.0])
    Rf = Rc @ Ry(np.radians(-FEM0) + qf)
    tf = tc + Rc @ np.array([L1, 0.0, -FEM_DROP])
    Rt = Rf @ Ry(np.radians(180.0 - KNEE0) + qk)
    tt = tf + Rf @ np.array([L2, 0.0, 0.0])
    foot = tt + Rt @ np.array([L3, 0.0, 0.0])
    return (Rc, tc), (Rf, tf), (Rt, tt), foot


def seg_d(P, a, b):
    ab = b - a
    t = np.clip((P - a) @ ab / max(1e-12, ab @ ab), 0.0, 1.0)
    return np.linalg.norm(P - (a[None] + t[:, None] * ab[None]), axis=1)


# chassis = TWO TIGHT boxes: plate slab + controller/sonar stack
BOXES = [
    (np.array([-0.063, -0.076, -0.034]), np.array([0.063, 0.059, 0.006])),
    (np.array([-0.042, -0.035, 0.006]), np.array([0.042, 0.080, 0.062])),
]


def box_d(P):
    ds = []
    for lo, hi in BOXES:
        out = np.maximum(np.maximum(lo - P, P - hi), 0.0)
        ds.append(np.linalg.norm(out, axis=1))
    return np.minimum(*ds)


def leg_segments(i, q):
    (Rc, tc), (Rf, tf), (Rt, tt), foot = chain(i, *q)
    return [(tc, tf), (tf, tt), (tt, foot)]


def d25(P, a, b):
    return float(np.percentile(seg_d(P, a, b), 25))


# link surface radii: the hip servos live INSIDE the plate-slab box
# footprint, so raw axis-distance lost every vote to the box's zero
# inside-distance and hips/femurs rendered welded to the body (Bill,
# watching the walk: "some of his leg parts dont move when the others
# do. particularly, his femurs and hips"). Scoring legs by distance to
# the link SURFACE (axis minus radius, clamped) with ties going to the
# LEG lets a servo hugging its own axis escape the slab.
LINK_R = {0: 0.016, 1: 0.018, 2: 0.012}


def assign(qs):
    segs = [leg_segments(i, qs[i]) for i in range(6)]
    labels = []
    for o in range(NOBJ):
        P = samples[o]
        cd = float(np.percentile(box_d(P), 25))
        best, bd = ("chassis",), cd
        for i in range(6):
            for k, (a, b) in enumerate(segs[i]):
                d = max(0.0, d25(P, a, b) - LINK_R[k])
                if d <= bd:            # ties break toward the leg
                    bd, best = d, (i, k)
        labels.append((best, bd))
    return labels


def refit(i, objs, q0):
    by_link = {0: [], 1: [], 2: []}
    for o, k in objs:
        by_link[k].append(samples[o])
    P = {}
    for k, v in by_link.items():
        if not v:
            P[k] = None
            continue
        pts = np.concatenate(v)
        if len(pts) > 800:
            pts = pts[:: len(pts) // 800 + 1]
        P[k] = pts

    def cost(q):
        segs = leg_segments(i, q)
        c, n = 0.0, 0
        for k in range(3):
            if P[k] is None:
                continue
            d = seg_d(P[k], *segs[k])
            c += float(d.sum())
            n += len(d)
        return c / max(1, n)

    best_q, best_c = tuple(q0), cost(q0)
    span, step = np.radians(60), np.radians(8)
    for _round in range(3):
        qc0, qf0, qk0 = best_q
        for qc in np.arange(qc0 - span, qc0 + span + 1e-9, step):
            for qf in np.arange(qf0 - span, qf0 + span + 1e-9, step):
                for qk in np.arange(qk0 - span, qk0 + span + 1e-9, step):
                    c = cost((qc, qf, qk))
                    if c < best_c:
                        best_c, best_q = c, (qc, qf, qk)
        span, step = step * 1.5, step / 4.0
    return best_q, best_c


# INIT: azimuth-lobe round (unsticks the EM from the zero-pose trap)
qs = [(0.0, 0.0, 0.0)] * 6
cen_all = np.array([s.mean(axis=0) for s in samples])
outside = box_d(cen_all) > 0.004
mount_az = [np.arctan2(ry, rx) for rx, ry in RADIAL]
for i in range(6):
    objs = []
    for o in range(NOBJ):
        if not outside[o]:
            continue
        az = np.arctan2(cen_all[o][1], cen_all[o][0])
        dz = np.degrees(abs((az - mount_az[i] + np.pi) % (2 * np.pi) - np.pi))
        if dz > 32:
            continue
        segs = leg_segments(i, qs[i])
        k = int(np.argmin([d25(samples[o], a, b) for a, b in segs]))
        objs.append((o, k))
    if objs:
        q, c = refit(i, objs, qs[i])
        qs[i] = q
        print("init leg %d: %3d lobe objs  q=(%6.1f,%6.1f,%6.1f)deg  d %.1fmm"
              % (i, len(objs), np.degrees(q[0]), np.degrees(q[1]),
                 np.degrees(q[2]), c * 1000), flush=True)

for it in range(3):
    labels = assign(qs)
    counts = sum(1 for (lab, _) in labels if lab != ("chassis",))
    print("iter %d: %d objects on legs" % (it, counts), flush=True)
    new_qs = []
    for i in range(6):
        objs = [(o, lab[1]) for o, (lab, _) in enumerate(labels)
                if lab != ("chassis",) and lab[0] == i]
        if not objs:
            new_qs.append(qs[i])
            continue
        q, c = refit(i, objs, qs[i])
        new_qs.append(q)
        print("  leg %d: %3d objs  q=(%6.1f,%6.1f,%6.1f)deg  mean-d %.1fmm"
              % (i, len(objs), np.degrees(q[0]), np.degrees(q[1]),
                 np.degrees(q[2]), c * 1000), flush=True)
    qs = new_qs

labels = assign(qs)
out = {"pose_deg": {str(i): [float(np.degrees(a)) for a in qs[i]]
                    for i in range(6)},
       "labels": [], "resid_mm": []}
LINK = {0: "coxa", 1: "femur", 2: "tibia"}
for o, (lab, d) in enumerate(labels):
    name = "chassis" if lab == ("chassis",) else "%s%d" % (LINK[lab[1]], lab[0])
    out["labels"].append(name)
    out["resid_mm"].append(round(d * 1000, 1))
json.dump(out, open(os.path.join(SP, "fit_result.json"), "w"))
r = np.array(out["resid_mm"])
print("residuals: median %.1fmm  p90 %.1fmm  max %.1fmm"
      % (np.median(r), np.percentile(r, 90), r.max()))
print("fit_result.json written")
