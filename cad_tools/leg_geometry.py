"""Pass 2: derive Gary's leg geometry from the Rhino model.
Servo shaft axes from (hub centroid, horn-disc normal); legs by proximity; link lengths as axis-to-axis
distances (pose-invariant); foot tip = farthest vertex from the tibia axis among that leg's lower parts.
Run: uv run --with rhino3dm --with numpy python leg_geometry.py"""
import rhino3dm, numpy as np, json, os, math
HERE=os.path.dirname(os.path.abspath(__file__))
PATH = r"C:\Users\BillH\Desktop\NERD\DROIDS!!\GARY\3. Technical Parameter & Diagram\1. Diagram\minihexa3 003.3dm"
d=json.load(open(os.path.join(HERE,"out","parts.json"),encoding="utf-8")); objs=d["objects"]
def sel(t):
    return [o for o in objs if "pca_ext" in o and all(abs(o["pca_ext"][i]-t[i])<=0.6 for i in range(3))]
bodies=sel((13,14,32.5)); discK=sel((4.5,20,20)); hubs=sel((8,10,10))
assert len(bodies)==len(discK)==len(hubs)==18
C=lambda L: np.array([o["centroid"] for o in L])
Hc=C(hubs); Dc=C(discK)
servos=[]
for b in bodies:
    bc=np.array(b["centroid"]); h=int(np.linalg.norm(Hc-bc,axis=1).argmin()); k=int(np.linalg.norm(Dc-bc,axis=1).argmin())
    n=np.array(discK[k]["pca_axes"][0]); n/=np.linalg.norm(n)
    p=Hc[h]
    if n@(p-Dc[k])<0: n=-n          # point from horn disc toward hub (out of the servo face)
    servos.append(dict(body=bc,p=p,n=n,long=np.array(b["pca_axes"][2])))
def line_dist(p1,n1,p2,n2):
    c=np.cross(n1,n2); s=np.linalg.norm(c)
    if s<1e-3: return float(np.linalg.norm(np.cross(p2-p1,n1))), 0.0
    return abs(float((p2-p1)@c/s)), math.degrees(math.asin(min(1,s)))
coxa=[s for s in servos if abs(s["n"][2])>0.8]; other=[s for s in servos if abs(s["n"][2])<=0.8]
print("coxa servos:",len(coxa),"others:",len(other))
# assign others to nearest coxa
legs=[]
for c in coxa:
    mine=sorted(other,key=lambda s: np.linalg.norm(s["p"]-c["p"]))[:2]
    mine=sorted(mine,key=lambda s: line_dist(c["p"],c["n"],s["p"],s["n"])[0])
    legs.append(dict(coxa=c,femur=mine[0],tibia=mine[1]))
# sanity: each other assigned once
ids=[id(l["femur"]) for l in legs]+[id(l["tibia"]) for l in legs]; assert len(set(ids))==12
# order legs by azimuth of coxa axis (CCW from +x)
for l in legs: l["az"]=math.degrees(math.atan2(l["coxa"]["p"][1],l["coxa"]["p"][0]))
legs.sort(key=lambda l:l["az"])
# ---- mesh pass for foot tips ----
print("reading model for foot vertices...",flush=True)
m=rhino3dm.File3dm.Read(PATH)
def verts(g):
    meshes=[]
    if isinstance(g, rhino3dm.Brep):
        for i in range(len(g.Faces)):
            mm=g.Faces[i].GetMesh(rhino3dm.MeshType.Any)
            if mm: meshes.append(mm)
    elif isinstance(g,(rhino3dm.Mesh,)): meshes=[g]
    elif isinstance(g, rhino3dm.Extrusion):
        mm=g.GetMesh(rhino3dm.MeshType.Any); meshes=[mm] if mm else []
    pts=[]
    for mm in meshes:
        vs=mm.Vertices; pts.extend((vs[i].X,vs[i].Y,vs[i].Z) for i in range(len(vs)))
    return np.array(pts) if pts else None
allobjs=list(m.Objects)
tib_pts=[l["tibia"]["p"] for l in legs]
report=[]
for li,l in enumerate(legs):
    cx,fm,tb=l["coxa"],l["femur"],l["tibia"]
    L1,a1=line_dist(cx["p"],cx["n"],fm["p"],fm["n"])
    L2,a2=line_dist(fm["p"],fm["n"],tb["p"],tb["n"])
    # femur-axis point nearest the coxa axis (to get z offset of femur axis vs coxa hub)
    # closest points between coxa line and femur line
    w=cx["p"]-fm["p"]; a=cx["n"]@cx["n"]; b=cx["n"]@fm["n"]; c=fm["n"]@fm["n"]; dd=cx["n"]@w; e=fm["n"]@w; den=a*c-b*b
    sc=(b*e-c*dd)/den; tc=(a*e-b*dd)/den
    Pc=cx["p"]+sc*cx["n"]; Pf=fm["p"]+tc*fm["n"]
    # femur->tibia axis vector in the plane perpendicular to femur axis
    v=tb["p"]-Pf; v_perp=v-(v@fm["n"])*fm["n"]
    # gather this leg's lower parts and find farthest vertex from tibia axis
    best=None; bestd=0
    for o in allobjs:
        try: bb=o.Geometry.GetBoundingBox()
        except Exception: continue
        cen=np.array([(bb.Min.X+bb.Max.X)/2,(bb.Min.Y+bb.Max.Y)/2,(bb.Min.Z+bb.Max.Z)/2])
        # belongs to this leg: nearest tibia servo is ours, within 100mm, and not above the coxa
        dists=[np.linalg.norm(cen-tp) for tp in tib_pts]
        if int(np.argmin(dists))!=li or dists[li]>100 or cen[2]>cx["p"][2]+5: continue
        P=verts(o.Geometry)
        if P is None: continue
        rel=P-tb["p"]; perp=rel-np.outer(rel@tb["n"],tb["n"]); dist=np.linalg.norm(perp,axis=1)
        j=int(dist.argmax())
        if dist[j]>bestd: bestd=dist[j]; best=P[j]
    tip=best
    # posed angles: femur link angle vs horizontal, tibia link (tibia axis point->tip) 
    femur_vec=v_perp; fem_pitch=math.degrees(math.atan2(femur_vec[2],np.linalg.norm(femur_vec[:2])))
    tibia_vec=tip-tb["p"]; tibia_perp=tibia_vec-(tibia_vec@tb["n"])*tb["n"]
    tib_pitch=math.degrees(math.atan2(tibia_perp[2],np.linalg.norm(tibia_perp[:2])))
    knee_angle=math.degrees(math.acos(np.clip((femur_vec@tibia_perp)/np.linalg.norm(femur_vec)/np.linalg.norm(tibia_perp),-1,1)))
    r=dict(leg=li,azimuth_deg=round(l["az"],2),coxa_axis_xy=np.round(cx["p"][:2],2).tolist(),coxa_hub_z=round(float(cx["p"][2]),2),
           coxa_axis_dir=np.round(cx["n"],3).tolist(),
           L1_coxa_axis_to_femur_axis=round(L1,2),coxa_femur_axis_angle=round(a1,1),
           femur_axis_z_at_closest=round(float(Pf[2]),2),femur_axis_dir=np.round(fm["n"],3).tolist(),
           L2_femur_axis_to_tibia_axis=round(L2,2),femur_tibia_axis_angle=round(a2,1),
           tibia_axis_pt=np.round(tb["p"],2).tolist(),tibia_axis_dir=np.round(tb["n"],3).tolist(),
           L3_tibia_axis_to_foot_tip=round(bestd,2),foot_tip=np.round(tip,2).tolist(),
           posed_femur_pitch_deg=round(fem_pitch,1),posed_tibia_pitch_deg=round(tib_pitch,1),posed_knee_angle_deg=round(knee_angle,1),
           coxa_to_foot_horiz=round(float(np.linalg.norm((tip-Pc)[:2])),2))
    report.append(r); print(json.dumps(r))
json.dump(report,open(os.path.join(HERE,"out","leg_geometry.json"),"w"),indent=1)
# summary
def stat(k): a=np.array([r[k] for r in report]); return f"{a.mean():.2f} ± {a.std():.2f} (min {a.min():.2f} max {a.max():.2f})"
print("\nSUMMARY (mm)")
for k in ("L1_coxa_axis_to_femur_axis","L2_femur_axis_to_tibia_axis","L3_tibia_axis_to_foot_tip","coxa_hub_z","femur_axis_z_at_closest","posed_femur_pitch_deg","posed_tibia_pitch_deg","posed_knee_angle_deg","coxa_to_foot_horiz"):
    print(f"  {k:32s} {stat(k)}")
print("  coxa axis radius:", [round(float(np.hypot(*r['coxa_axis_xy'])),2) for r in report])
print("  azimuths:", [r['azimuth_deg'] for r in report])
