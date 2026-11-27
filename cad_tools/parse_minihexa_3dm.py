import rhino3dm
from collections import Counter, defaultdict

PATH = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\3. Technical Parameter & Diagram\1. Diagram\minihexa3 003.3dm"

print("reading (494MB, may take a while)...", flush=True)
m = rhino3dm.File3dm.Read(PATH)
print("units:", m.Settings.ModelUnitSystem, flush=True)
print("objects:", len(m.Objects), " layers:", len(m.Layers), " instance defs:", len(m.InstanceDefinitions), flush=True)

layers = {l.Index: l.FullPath for l in m.Layers}
print("\n--- LAYERS (first 80) ---")
for l in list(m.Layers)[:80]:
    print(f"  [{l.Index}] {l.FullPath}")

def bbox_dims(g):
    try:
        bb = g.GetBoundingBox()
        return sorted([bb.Max.X - bb.Min.X, bb.Max.Y - bb.Min.Y, bb.Max.Z - bb.Min.Z])
    except Exception:
        return None

# Instance definitions: canonical geometry, un-posed
print("\n--- INSTANCE DEFINITIONS ---")
idefs = list(m.InstanceDefinitions)
# count instances per definition
inst_count = Counter()
for obj in m.Objects:
    g = obj.Geometry
    if isinstance(g, rhino3dm.InstanceReference):
        inst_count[str(g.ParentIdefId)] += 1

for idef in idefs:
    ids = idef.GetObjectIds()
    # union bbox over the definition's own geometry
    mn = [1e18]*3; mx = [-1e18]*3
    found = False
    id_set = set(str(i) for i in ids)
    for obj in m.Objects:
        if str(obj.Attributes.Id) in id_set:
            try:
                bb = obj.Geometry.GetBoundingBox()
            except Exception:
                continue
            found = True
            mn = [min(mn[0], bb.Min.X), min(mn[1], bb.Min.Y), min(mn[2], bb.Min.Z)]
            mx = [max(mx[0], bb.Max.X), max(mx[1], bb.Max.Y), max(mx[2], bb.Max.Z)]
    dims = sorted(round(a-b, 2) for a, b in zip(mx, mn)) if found else None
    n = inst_count.get(str(idef.Id), 0)
    print(f"  '{idef.Name}' | parts:{len(ids)} | instances:{n} | dims:{dims}")

# Fallback: cluster raw-object bboxes in servo-ish size range
print("\n--- RAW OBJECT BBOX CLUSTERS (5-60mm, count>=6) ---")
clusters = Counter()
sample_layer = {}
for obj in m.Objects:
    d = bbox_dims(obj.Geometry)
    if not d or d[0] < 5 or d[2] > 60:
        continue
    key = tuple(round(x, 1) for x in d)
    clusters[key] += 1
    sample_layer[key] = layers.get(obj.Attributes.LayerIndex, "?")
for key, n in clusters.most_common(40):
    if n >= 6:
        print(f"  {n:4d} x  dims {key}  layer='{sample_layer[key]}'")
print("done", flush=True)
