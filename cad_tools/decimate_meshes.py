"""Decimate the exported CAD meshes to something MuJoCo will load and a
repo will tolerate.

The raw export is faithful but absurd: 448k triangles on the chassis and
240 MB total, and MuJoCo refuses any STL over 200k faces. This does
vertex-clustering decimation -- snap vertices to a grid, weld duplicates,
drop the faces that collapse -- shrinking the grid until the triangle
count is under target. Shape is preserved at the millimetre scale, which
is all a spectator view needs.

Run: uv run --with numpy python decimate_meshes.py [target_tris]
"""
import os, struct, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MESHDIR = os.path.join(os.path.dirname(HERE), "sim", "meshes")
TARGET = int(sys.argv[1]) if len(sys.argv) > 1 else 24000


def read_stl(path):
    with open(path, "rb") as fh:
        fh.read(80)
        n = struct.unpack("<I", fh.read(4))[0]
        buf = fh.read(n * 50)
    a = np.frombuffer(buf, dtype=np.uint8).reshape(n, 50)
    tri = a[:, 12:48].copy().view(np.float32).reshape(n, 3, 3).astype(np.float64)
    return tri


def write_stl(path, tri):
    n = len(tri)
    with open(path, "wb") as fh:
        fh.write(b"\0" * 80)
        fh.write(struct.pack("<I", n))
        for t in tri:
            p, q, r = t
            nv = np.cross(q - p, r - p)
            ln = np.linalg.norm(nv)
            nv = nv / ln if ln > 1e-12 else np.zeros(3)
            fh.write(struct.pack("<3f", *nv))
            for v in t:
                fh.write(struct.pack("<3f", *v))
            fh.write(b"\0\0")


def cluster(tri, grid):
    V = tri.reshape(-1, 3)
    key = np.floor(V / grid).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    # representative = mean of each cluster
    nclust = inv.max() + 1
    sums = np.zeros((nclust, 3))
    cnts = np.zeros(nclust)
    np.add.at(sums, inv, V)
    np.add.at(cnts, inv, 1)
    reps = sums / cnts[:, None]
    F = inv.reshape(-1, 3)
    ok = (F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2])
    F = F[ok]
    if len(F) == 0:
        return None
    return reps[F]


def main():
    total_before = total_after = 0
    for fn in sorted(os.listdir(MESHDIR)):
        if not fn.endswith(".stl"):
            continue
        path = os.path.join(MESHDIR, fn)
        tri = read_stl(path)
        before = len(tri)
        total_before += os.path.getsize(path)
        if before <= TARGET:
            print("  %-12s %7d tris (already small)" % (fn, before))
            total_after += os.path.getsize(path)
            continue
        span = float(np.ptp(tri.reshape(-1, 3), axis=0).max())
        grid = span / 90.0
        out = tri
        for _ in range(14):
            c = cluster(tri, grid)
            if c is None:
                break
            out = c
            if len(out) <= TARGET:
                break
            grid *= 1.35
        write_stl(path, out)
        total_after += os.path.getsize(path)
        print("  %-12s %7d -> %6d tris  (grid %.2f mm)"
              % (fn, before, len(out), grid * 1000))
    print("total %.1f MB -> %.1f MB" % (total_before / 1e6, total_after / 1e6))


if __name__ == "__main__":
    main()
