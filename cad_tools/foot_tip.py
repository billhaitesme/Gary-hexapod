"""Pass 3: foot tip + tibia decomposition + body plate heights. Uses out/leg_geometry.json (axes) and re-reads mesh verts."""
import rhino3dm, numpy as np, json, os, math
HERE=os.path.dirname(os.path.abspath(__file__))
PATH = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\3. Technical Parameter & Diagram\1. Diagram\minihexa3 003.3dm"
parts=json.load(open(os.path.join(HERE,"out","parts.json"),encoding="utf-8"))["objects"]
legs=json.load(open(os.path.join(HERE,"out","leg_geometry.json")))
m=rhino3dm.File3dm.Read(PATH); allobjs=list(m.Objects)
def verts(g):
    meshes=[]
    if isinstance(g, rhino3dm.Brep):
        for i in range(len(g.Faces)):
            mm=g.Faces[i].GetMesh(rhino3dm.MeshType.Any)
            if mm: meshes.append(mm)
    elif isinstance(g, rhino3dm.Extrusion):
        mm=g.GetMesh(rhino3dm.MeshType.Any); meshes=[mm] if mm else []
    pts=[]
    for mm in meshes:
        vs=mm.Vertices; pts.extend((vs[i].X,vs[i].Y,vs[i].Z) for i in range(len(vs)))
    return np.array(pts) if pts else None
def match(t,tol=0.6): return [o for o in parts if "pca_ext" in o and all(abs(o["pca_ext"][i]-t[i])<=tol for i in range(3))]
pads=match((3,8.8,9.2)); blades=match((1.5,28.6,85.9)); print(len(pads),len(blades))
out=[]
for l in legs:
    tp=np.array(l["tibia_axis_pt"]); tn=np.array(l["tibia_axis_dir"]); cx=np.array(l["coxa_axis_xy"]+[0])
    fz=l["femur_axis_z_at_closest"]
    # radial unit vector of this leg (from coxa axis outward, horizontal), leg plane = span(radial, z)
    # femur axis dir tn is horizontal & tangential; radial = z x tn (choose sign pointing away from body center (0,-6.3))
    rad=np.cross([0,0,1],tn); rad/=np.linalg.norm(rad)
    if rad@ (cx-np.array([0,-6.3,0]))<0: rad=-rad
    pad=min(pads,key=lambda o:np.linalg.norm(np.array(o["centroid"])-tp))
    blade=min(blades,key=lambda o:np.linalg.norm(np.array(o["centroid"])-tp))
    P=verts(allobjs[pad["i"]].Geometry); B=verts(allobjs[blade["i"]].Geometry)
    tip=P[P[:,2].argmin()]                       # lowest pad vertex (contact point in this pose)
    tip_c=P[np.abs(P[:,2]-P[:,2].min())<0.3].mean(axis=0)  # centre of the bottom face
    rel=tip_c-tp; along_axis=float(rel@tn); r=rel-along_axis*tn
    L3=float(np.linalg.norm(r)); out_h=float(r@rad); down=float(-r[2])
    # blade farthest point from tibia axis (mechanical tibia length)
    relB=B-tp; perpB=relB-np.outer(relB@tn,tn); dB=np.linalg.norm(perpB,axis=1); Lblade=float(dB.max())
    # femur: tibia axis relative to femur axis in leg plane
    fem_out=float((tp-np.array([*cx[:2],0]))@rad) - l["L1_coxa_axis_to_femur_axis"]; fem_up=float(tp[2]-fz)
    rec=dict(leg=l["leg"],L3_axis_to_pad_bottom_centre=round(L3,2),tibia_out_horiz=round(out_h,2),tibia_down=round(down,2),
             tibia_pitch_from_vertical_deg=round(math.degrees(math.atan2(out_h,down)),1),foot_offset_along_axis=round(along_axis,2),
             blade_max_radius=round(Lblade,2),femur_out=round(fem_out,2),femur_up=round(fem_up,2),
             foot_below_femur_axis=round(fz-tip_c[2],2),foot_horiz_from_coxa_axis=round(float((tip_c-np.array([*cx[:2],0]))@rad),2),
             pad_bottom_z=round(float(P[:,2].min()),2))
    out.append(rec); print(rec)
# body plates: black sheet metal large parts near centre
print("\nBody sheet-metal parts (centroid within 40mm of centre XY):")
for o in parts:
    if "pca_ext" not in o: continue
    c=o["centroid"]
    if o["layer"]=="黑色钣金" and math.hypot(c[0],c[1]+6.3)<40:
        print("  ext",[round(x,1) for x in o["pca_ext"]],"z",o["aabb_min"][2],"..",o["aabb_max"][2],"c",np.round(c,1))
json.dump(out,open(os.path.join(HERE,"out","foot_tip.json"),"w"),indent=1)
a=np.array([[r["L3_axis_to_pad_bottom_centre"],r["tibia_pitch_from_vertical_deg"],r["blade_max_radius"],r["femur_out"],r["femur_up"],r["foot_offset_along_axis"]] for r in out])
print("\nmean L3, tibia pitch, blade radius, femur_out, femur_up, along-axis offset:", a.mean(axis=0).round(2), "std", a.std(axis=0).round(2))
