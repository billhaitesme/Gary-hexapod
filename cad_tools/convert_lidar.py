"""Convert Bill's LD19 lidar STEP (his add-on -- the vendor Rhino model
has no lidar) into sim/meshes/lidar_d.stl, fully placed in base-body
coordinates. Encodes every placement decision Bill dictated live:

  * centered on the sim lidar mount (0, 0.0555) -- his printed riser
    is unmeasured, so it floats there ("a floating top hat for now")
  * spun 90 deg CLOCKWISE from the STEP's native orientation ("the
    ZERO facing for the lidar is actually to the right... spin the
    LiDAR 90 degrees clockwise")
  * lowered so the ROTOR CENTER sits on the ray plane z=0.0588 ("the
    LiDAR fires from about the center of its cylindrical section...
    its base needs to come down a bit until your rays line up")

Run: uv run --with cascadio --with trimesh --with numpy python convert_lidar.py
"""
import os
import numpy as np

import cascadio
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
STEP = os.path.join(HERE, "LD19_v2.step")
GLB = os.path.join(HERE, "out", "ld19.glb")
OUT = os.path.join(os.path.dirname(HERE), "sim", "meshes", "lidar_d.stl")

MOUNT_Y = 0.0555          # sim lidar mount centerline
RAY_Z = 0.0588            # rangefinder site plane
BASE_Z = 0.042            # second-tier deck (pre-riser)
ROTOR_DROP = 0.0062       # rotor-center-to-ray-plane, measured on the
                          # mesh radius profile (rotor band ~z 0.065)

cascadio.step_to_glb(STEP, GLB, tol_linear=0.15)
m = trimesh.load(GLB, force="mesh")
b = m.bounds
cx, cy = (b[0][0] + b[1][0]) / 2, (b[0][1] + b[1][1]) / 2
m.apply_translation([-cx, MOUNT_Y - cy, BASE_Z - b[0][2]])
# 90 deg clockwise about +z through the mount centerline
T = trimesh.transformations.rotation_matrix(
    -np.pi / 2, [0, 0, 1], [0, MOUNT_Y, 0])
m.apply_transform(T)
m.apply_translation([0, 0, -ROTOR_DROP])
m.export(OUT)
print("lidar_d.stl written:", len(m.faces), "tris, bounds",
      np.round(m.bounds, 4).tolist())
print("NOTE: decimate_meshes.py caps it at 24k tris afterwards")
