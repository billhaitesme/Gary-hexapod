"""Export Gary's REAL geometry into MuJoCo meshes -- v5, fit-driven.

Bill: "you have the 3-D files for his entire build... can you not make
him look like he actually looks? i know its practically not important,
but... it is kool."

Pipeline (each step has its own script):
  1. cache_objects.py  -- 494 MB Rhino model -> out/cad_objects.npz
  2. fit_assign.py     -- EM fit: posed joint angles + per-object
                          part assignment -> out/fit_result.json
  3. THIS script       -- place every object by the EXACT inverse of
                          its link's FK chain transform at the fitted
                          pose; group into '<body>_<tone>' STLs (tones
                          = the vendor's COLOR layers: silver, black,
                          white, copper, orange feet)
  4. decimate_meshes.py -- vertex-cluster to MuJoCo-friendly sizes

History, for whoever touches this next: v1-v4 assigned parts by
distance to a hand-derived skeleton whose foot_tip points were ~69 mm
off the posed feet. Chassis plates landed inside tibia files, coxa
brackets welded into the chassis at their posed stance ("theres shit
everywhere! hahaha that looks like it exploded" -- Bill, and later
"those parts did NOT go back together the way you took them apart").
The EM fit killed the skeleton dependence entirely; v5's render put
the sonar eyes on the front face and the rubber caps exactly on the
collision feet.

The meshes are VISUAL ONLY and their materials carry alpha=0 in the
model: MuJoCo rangefinders skip zero-alpha geoms (measured with a
6-case ray probe; group filtering does NOT work), so lidar physics is
bit-identical by construction; gary_mujoco's render shim flips alpha
on only while a frame is drawn.

Run: uv run --with "mujoco<3.12" --with numpy python export_meshes.py
"""
import json, os, struct, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "sim"))
import gary_mujoco as G

SP = os.path.join(HERE, "out")
OUT = os.path.join(os.path.dirname(HERE), "sim", "meshes")
L1, L2, L3 = G.L1, G.L2, G.L3
FEM_DROP, FEM0, KNEE0 = G.FEM_DROP, G.FEM0, G.KNEE0
MOUNTS, RADIAL = G.MOUNTS, G.RADIAL
Y_OFF, Z_OFF = 6.30, 58.42
# vendor layer -> tone (layers are COLORS: 0 silver, 2 copper, 4
# yellow, 5-8 blacks, 9 orange rubber feet, 10 white, ...)
LAYER_TONE = {0: "m", 1: "m", 2: "c", 3: "d", 4: "y", 5: "d", 6: "d",
              7: "d", 8: "d", 9: "o", 10: "w", 11: "d"}

fit = json.load(open(os.path.join(SP, "fit_result.json")))
z = np.load(os.path.join(SP, "cad_objects.npz"))
V, F, voff, foff, layer = z["V"], z["F"], z["voff"], z["foff"], z["layer"]
NOBJ = len(voff) - 1
labels = fit["labels"]
pose = {int(i): [np.radians(a) for a in q] for i, q in fit["pose_deg"].items()}


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
    return {"coxa": (Rc, tc), "femur": (Rf, tf), "tibia": (Rt, tt)}


frames = {}
for i in range(6):
    for part, (R, t) in chain(i, *pose[i]).items():
        frames["%s%d" % (part, i)] = (R, t)

buckets = {}
for o in range(NOBJ):
    lab = labels[o]
    tone = LAYER_TONE.get(int(layer[o]), "m")
    P = V[voff[o]:voff[o + 1]].astype(np.float64)
    # 180-deg yaw: the hex mount ring is centro-symmetric, so hubs
    # match the sim mounts to 0.03 mm BOTH ways -- the sonar face
    # broke the tie (Bill: 'the red dot, and subsequently the LiDAR
    # are on the wrong side'). Sim forward = +y = his face.
    P = np.column_stack([-P[:, 0], -(P[:, 1] + Y_OFF), P[:, 2] - Z_OFF]) / 1000.0
    if lab != "chassis":
        R, t = frames[lab]
        P = (P - t) @ R          # local = R^T (p - t)
    Fo = F[foff[o]:foff[o + 1]].astype(np.int64)
    buckets.setdefault((lab, tone), []).append((P, Fo))


def write_stl(path, tris):
    with open(path, "wb") as fh:
        fh.write(b"\0" * 80)
        fh.write(struct.pack("<I", len(tris)))
        for t in tris:
            p, q, r = t
            nv = np.cross(q - p, r - p)
            ln = np.linalg.norm(nv)
            nv = nv / ln if ln > 1e-12 else np.zeros(3)
            fh.write(struct.pack("<3f", *nv))
            for v in t:
                fh.write(struct.pack("<3f", *v))
            fh.write(b"\0\0")


# wipe previous robot STLs (keep lidar_d -- Bill's LD19 STEP add-on,
# converted separately; it is not in the vendor model)
for fn in os.listdir(OUT):
    if fn.endswith(".stl") and not fn.startswith("lidar"):
        os.remove(os.path.join(OUT, fn))

summary = {}
for (lab, tone), parts in sorted(buckets.items()):
    T = np.concatenate([P[Fo] for P, Fo in parts])
    name = "%s_%s" % (lab, tone)
    path = os.path.join(OUT, name + ".stl")
    write_stl(path, T)
    summary[name] = dict(tris=int(len(T)),
                         kb=round(os.path.getsize(path) / 1024, 1))
    print("  %-12s %7d tris" % (name, len(T)), flush=True)
if os.path.exists(os.path.join(OUT, "lidar_d.stl")):
    summary["lidar_d"] = dict(tris=0, kb=round(
        os.path.getsize(os.path.join(OUT, "lidar_d.stl")) / 1024, 1))
json.dump(summary, open(os.path.join(OUT, "manifest.json"), "w"), indent=1)
print("wrote %d buckets + manifest" % len(summary))
