import rhino3dm
import numpy as np
from collections import Counter, defaultdict

PATH = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\3. Technical Parameter & Diagram\1. Diagram\minihexa3 003.3dm"

print("reading...", flush=True)
m = rhino3dm.File3dm.Read(PATH)
layers = {l.Index: l.FullPath for l in m.Layers}

def get_vertices(g):
    meshes = []
    if isinstance(g, rhino3dm.Mesh):
        meshes = [g]
    elif isinstance(g, rhino3dm.Brep):
        for i in range(len(g.Faces)):
            mm = g.Faces[i].GetMesh(rhino3dm.MeshType.Any)
            if mm:
                meshes.append(mm)
    elif isinstance(g, rhino3dm.Extrusion):
        mm = g.GetMesh(rhino3dm.MeshType.Any)
        if mm:
            meshes = [mm]
    pts = []
    for mm in meshes:
        vs = mm.Vertices
        for i in range(len(vs)):
            v = vs[i]
            pts.append((v.X, v.Y, v.Z))
    return np.array(pts) if pts else None

def obb_dims(pts):
    if len(pts) < 8:
        return None
    c = pts - pts.mean(axis=0)
    cov = np.cov(c.T)
    _, vecs = np.linalg.eigh(cov)
    proj = c @ vecs
    ext = proj.max(axis=0) - proj.min(axis=0)
    return sorted(float(x) for x in ext)

clusters = Counter()
info = defaultdict(list)
n_done = 0
for obj in m.Objects:
    pts = get_vertices(obj.Geometry)
    if pts is None:
        continue
    aabb = pts.max(axis=0) - pts.min(axis=0)
    if max(aabb) > 80 or max(aabb) < 8:
        continue  # only mid-size parts
    d = obb_dims(pts)
    if d is None:
        continue
    n_done += 1
    key = tuple(round(x * 2) / 2 for x in d)  # 0.5mm bins
    clusters[key] += 1
    info[key].append(layers.get(obj.Attributes.LayerIndex, "?"))

print(f"parts measured: {n_done}")
print("\n--- ORIENTED (PCA) DIMS CLUSTERS, count>=4 ---")
for key, n in sorted(clusters.items(), key=lambda kv: -kv[1]):
    if n >= 4:
        lays = Counter(info[key]).most_common(2)
        print(f"  {n:4d} x  {key}  layers={lays}")

print("\n--- SERVO-RANGE candidates (short 9-16, mid 15-28, long 22-42), any count ---")
for key, n in sorted(clusters.items(), key=lambda kv: -kv[1]):
    s, md, lg = key
    if 9 <= s <= 16 and 15 <= md <= 28 and 22 <= lg <= 42:
        lays = Counter(info[key]).most_common(2)
        print(f"  {n:4d} x  {key}  layers={lays}")
print("done", flush=True)
