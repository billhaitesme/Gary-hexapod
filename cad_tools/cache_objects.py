"""Read the 494 MB vendor Rhino model ONCE into a fast npz cache
(cad_tools/out/cad_objects.npz, gitignored -- regenerate on demand) so
the fit/export loop iterates in seconds, not 5-minute reads.

Run: uv run --with rhino3dm --with numpy python cache_objects.py
"""
import os
import numpy as np
import rhino3dm

HERE = os.path.dirname(os.path.abspath(__file__))
PATH = os.path.join(os.path.dirname(HERE), "3. Technical Parameter & Diagram",
                    "1. Diagram", "minihexa3 003.3dm")
OUT = os.path.join(HERE, "out", "cad_objects.npz")

print("reading %.0f MB..." % (os.path.getsize(PATH) / 1e6), flush=True)
m = rhino3dm.File3dm.Read(PATH)


def obj_mesh(g):
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
            meshes.append(mm)
    if not meshes:
        return None
    V, F = [], []
    for mm in meshes:
        base = len(V)
        vs = mm.Vertices
        V.extend((vs[i].X, vs[i].Y, vs[i].Z) for i in range(len(vs)))
        fs = mm.Faces
        for i in range(len(fs)):
            f = fs[i]
            a, b, c, d = f[0], f[1], f[2], f[3]
            F.append((base + a, base + b, base + c))
            if d != c:
                F.append((base + a, base + c, base + d))
    if not V or not F:
        return None
    return np.asarray(V, dtype=np.float32), np.asarray(F, dtype=np.int32)


Vs, Fs, voff, foff, layers = [], [], [0], [0], []
n = 0
for obj in m.Objects:
    r = obj_mesh(obj.Geometry)
    if r is None:
        continue
    V, F = r
    Vs.append(V)
    Fs.append(F)
    voff.append(voff[-1] + len(V))
    foff.append(foff[-1] + len(F))
    layers.append(obj.Attributes.LayerIndex)
    n += 1
    if n % 200 == 0:
        print("  ...%d" % n, flush=True)

np.savez_compressed(OUT,
                    V=np.concatenate(Vs), F=np.concatenate(Fs),
                    voff=np.asarray(voff, dtype=np.int64),
                    foff=np.asarray(foff, dtype=np.int64),
                    layer=np.asarray(layers, dtype=np.int32))
print("cached %d objects -> %s (%.0f MB)"
      % (n, OUT, os.path.getsize(OUT) / 1e6), flush=True)
