"""Pass 1: read the 494MB Rhino model once, dump per-object summary + PCA frame to parts.json.
Run: uv run --with rhino3dm --with numpy python dump_parts.py"""
import rhino3dm, numpy as np, json, sys, os
PATH = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\3. Technical Parameter & Diagram\1. Diagram\minihexa3 003.3dm"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out", "parts.json")
os.makedirs(os.path.dirname(OUT), exist_ok=True)
print("reading...", flush=True)
m = rhino3dm.File3dm.Read(PATH)
print("units:", m.Settings.ModelUnitSystem, "objects:", len(m.Objects), "layers:", len(m.Layers), "idefs:", len(m.InstanceDefinitions), flush=True)
layers = {l.Index: l.FullPath for l in m.Layers}
def verts(g):
    meshes = []
    if isinstance(g, rhino3dm.Mesh): meshes=[g]
    elif isinstance(g, rhino3dm.Brep):
        for i in range(len(g.Faces)):
            mm = g.Faces[i].GetMesh(rhino3dm.MeshType.Any)
            if mm: meshes.append(mm)
    elif isinstance(g, rhino3dm.Extrusion):
        mm = g.GetMesh(rhino3dm.MeshType.Any)
        if mm: meshes=[mm]
    pts=[]
    for mm in meshes:
        vs=mm.Vertices
        pts.extend((vs[i].X,vs[i].Y,vs[i].Z) for i in range(len(vs)))
    return np.array(pts) if pts else None
rows=[]
for k,obj in enumerate(m.Objects):
    g=obj.Geometry
    r={"i":k,"id":str(obj.Attributes.Id),"name":obj.Attributes.Name or "","layer":layers.get(obj.Attributes.LayerIndex,"?"),
       "type":type(g).__name__}
    if isinstance(g, rhino3dm.InstanceReference):
        r["idef"]=str(g.ParentIdefId); x=g.Xform
        r["xform"]=[[x.M00,x.M01,x.M02,x.M03],[x.M10,x.M11,x.M12,x.M13],[x.M20,x.M21,x.M22,x.M23]]
    p=verts(g)
    if p is not None and len(p)>=4:
        c=p.mean(axis=0); q=p-c
        cov=np.cov(q.T); w,v=np.linalg.eigh(cov)  # ascending
        proj=q@v; ext=proj.max(axis=0)-proj.min(axis=0)
        r.update(n=len(p),centroid=c.round(3).tolist(),aabb_min=p.min(axis=0).round(3).tolist(),aabb_max=p.max(axis=0).round(3).tolist(),
                 pca_ext=ext.round(3).tolist(),pca_axes=v.T.round(5).tolist(),  # rows = axes, ascending extent
                 obb_center=(c+v@((proj.max(axis=0)+proj.min(axis=0))/2)).round(3).tolist())
    rows.append(r)
    if k%100==0: print(k, flush=True)
json.dump({"layers":layers,"idefs":[{"id":str(d.Id),"name":d.Name,"n":len(d.GetObjectIds())} for d in m.InstanceDefinitions],"objects":rows},open(OUT,"w"),indent=0)
print("wrote",OUT,len(rows))
