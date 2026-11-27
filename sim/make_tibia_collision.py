# Tibia collision from the CAD (Bill, 2026-08-23: "might as well use his
# actual geometry"). MuJoCo collides a mesh as its convex hull, which
# would fill the blade's bow; so each leg's blade mesh (tibia{i}_d.stl)
# is sliced along the shin into short slabs and each slab's convex hull
# becomes one collision mesh. Writes meshes/tibia{i}_col{k}.stl and a
# JSON list the XML builder reads.
import json
import os
import numpy as np
from stl import mesh
from scipy.spatial import ConvexHull

MESHDIR = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\sim\meshes"
# slab edges along the tibia axis (x, m): bracket, then the blade
EDGES = [-0.012, 0.025, 0.038, 0.050, 0.062, 0.080]
out = {}
for i in range(6):
    fn = os.path.join(MESHDIR, "tibia%d_d.stl" % i)
    m = mesh.Mesh.from_file(fn)
    v = m.vectors.reshape(-1, 3)
    names = []
    for k in range(len(EDGES) - 1):
        x0, x1 = EDGES[k], EDGES[k + 1]
        sl = v[(v[:, 0] >= x0) & (v[:, 0] < x1)]
        if len(sl) < 12:
            continue
        # include a thin band of neighbours so slabs overlap by ~1 mm
        pts = v[(v[:, 0] >= x0 - 0.001) & (v[:, 0] < x1 + 0.001)]
        pts = np.unique(np.round(pts, 5), axis=0)
        if len(pts) < 4:
            continue
        try:
            hull = ConvexHull(pts)
        except Exception:
            continue
        tris = pts[hull.simplices]
        cm = mesh.Mesh(np.zeros(len(tris), dtype=mesh.Mesh.dtype))
        cm.vectors[:] = tris
        name = "tibia%d_col%d" % (i, k)
        cm.save(os.path.join(MESHDIR, name + ".stl"))
        names.append(name)
    out["tibia%d" % i] = names
    print("leg %d: %d collision slabs" % (i, len(names)))
json.dump(out, open(os.path.join(MESHDIR, "tibia_collision.json"), "w"), indent=1)
print("written meshes/tibia_collision.json")
