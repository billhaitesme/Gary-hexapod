"""Stage-2 sim: Gary as a MuJoCo digital twin, built from the CAD-extracted geometry.

* generates gary.xml (18 position-servo joints, CAD link lengths/mounts/pose)
* replays the GARY_v3 custom gait scheduler in real physics (gravity, contact, torque limits)
* metrics: distance, knee torque saturation, slip
* --view  : interactive viewer window (orbit/zoom while he walks)
* --video : offscreen render to mp4

Run:
  uv run --with mujoco --with numpy python gary_mujoco.py --gait WAVE --stick 1.0 --view
  uv run --with mujoco --with numpy --with imageio --with imageio-ffmpeg python gary_mujoco.py --video gary_wave.mp4

Masses (2026-08-18): component-sum ESTIMATE ~1.25 kg total — 18 micro servos ~290 g,
2x18650 ~94 g, 6000-series alu chassis (Hiwonder blog) + boards -> body 0.62 kg,
105 g/leg (30 coxa / 50 femur / 22 tibia / 3 foot). D500 LiDAR = 46 g CONFIRMED
(botland spec: 38.6 x 38.9 x 33.5 mm), mounted per Bill: second tier, centerline,
front face at the body's front edge, one body-thickness above the chassis box.
Servo = HPD-0127 CONFIRMED (0.176 N*m clutch limit, 1200 deg/s, 11 g); kp still an estimate.
NOVEMBER BENCH: weigh Gary + one leg; re-run train_gait.py after correcting.
Everything geometric is CAD ground truth (gary_geometry.h).
"""
import argparse, math, os, sys, time
import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))

# ---- CAD geometry (m) ----------------------------------------------------------
L1, L2, L3 = 0.02920, 0.03783, 0.07435
FEM_DROP   = 0.00408            # femur axis below coxa hub
COXA_Z     = 0.0503             # coxa hub above ground in the CAD pose
NEUTRAL_OUT, NEUTRAL_DOWN = 0.07107, 0.04605
FEM0, KNEE0 = 45.0, 56.7        # CAD pose: femur pitch up, knee interior angle
# Knee joint travel about KNEE0 (q = -(interior - KNEE0)). Bill, 2026-08-22:
# "his legs can straighten, and even invert" and there is no servo
# end-stop -- "his body will eventually get in the way". So the knee is
# free until self-collision: -160 => interior up to 216.7 deg (36.7 deg
# inverted); the IK itself tops out at 180. The old "-45 60" (interior
# <= 101.7 deg, hub-to-foot 90 mm vs 112 straight) was an unmeasured
# guess that every record before this date was set under
# (notes/WALL-DEADLOCK-2026-08-22.md). GARY_KNEE_RANGE="-45 60" rebuilds
# the old Gary for A/B.
KNEE_RANGE = os.environ.get("GARY_KNEE_RANGE", "-160 60")
# The knee actuator ctrlrange is DERIVED from the joint range: it used to
# be a fixed +-1.4 rad = a second, hidden clamp at 136.9 deg interior.
_kr = [float(x) for x in KNEE_RANGE.split()]
KNEE_CTRL = "%.4f %.4f" % (math.radians(_kr[0]), math.radians(_kr[1]))
KNEE_CMD_MAX = float(os.environ.get("GARY_KNEE_CMD_MAX", KNEE0 + 45.0))   # deg interior, see leg_ik
# Hips (coxa). Bill 2026-08-23: no servo stop -- "they can swing as far as
# the body, or another leg in the way allows them to". The old "-60 60"
# (and a fixed +-1.4 rad command) was unmeasured; every refused hook of
# 2026-08-22 failed on the controller's |c| < 1.0 rad gate. Self-collision
# is the real stop. GARY_COXA_RANGE="-60 60" rebuilds the old hips for A/B.
COXA_RANGE = os.environ.get("GARY_COXA_RANGE", "-90 90")
_cr = [float(x) for x in COXA_RANGE.split()]
COXA_CTRL = "%.4f %.4f" % (math.radians(_cr[0]), math.radians(_cr[1]))
COXA_CMD_MAX = math.radians(min(-_cr[0], _cr[1])) - 0.035   # rad, 2 deg inside the joint
# Femur. Same law (Bill 2026-08-23): no servo stop on any joint -- "if it
# clears, it can go there". The old "-80 80" + fixed +-1.4 rad command was
# unmeasured. GARY_FEMUR_RANGE="-80 80" rebuilds the old femur for A/B.
FEMUR_RANGE = os.environ.get("GARY_FEMUR_RANGE", "-150 150")
_fr = [float(x) for x in FEMUR_RANGE.split()]
FEMUR_CTRL = "%.4f %.4f" % (math.radians(_fr[0]), math.radians(_fr[1]))
# Foot friction. 2.0 = rubber caps (Bill, d92bca6 2026-08-22, unmeasured:
# "MEASURE IN SEPT"). The navigator book (PANIC 75.1 clean / 68.5 / explore
# 105.9 / race 103.2) was set at 1.2 and breaks at 2.0 (PANIC 123.3 s, 138
# wall hits) -- bisected 2026-08-22. GARY_FOOT_MU=1.2 rebuilds the book Gary.
FOOT_MU = os.environ.get("GARY_FOOT_MU", "2.0")
# Rung depth (Bill 2026-08-23: 'or, move the rungs' -- the September
# ladder is his to build; deeper bars bring the seat into the leg's
# absolute reach). Default 0.024 = the 24 mm bars every measurement
# used; GARY_RUNG_DEPTH=0.044 extends the bars 2 cm toward Gary. The
# bar's BACK edge stays at 0.429; the lip rides the front edge.
RUNG_DEPTH = float(os.environ.get("GARY_RUNG_DEPTH", "0.024"))
RUNG_Y0 = 0.429 - RUNG_DEPTH          # bar front face
RUNG_YC = (0.429 + RUNG_Y0) / 2.0     # bar centre
RUNG_YH = RUNG_DEPTH / 2.0            # bar y half-size
LIP_Y = RUNG_Y0 - 0.0025              # lip centre (overlapping the front edge by 1 mm, as built)
# mounts (x right, y forward) + neutral leg radial directions (unit)
MOUNTS = [( 0.04355,  0.0),  ( 0.03132,  0.0570), (-0.03132,  0.0570),
          (-0.04355,  0.0),  (-0.03132, -0.0570), ( 0.03132, -0.0570)]
RADIAL = [( 1, 0), ( .7071, .7071), (-.7071, .7071), (-1, 0), (-.7071, -.7071), ( .7071, -.7071)]

# Servo IDENTIFIED 2026-08-18: Hiwonder HPD-0127 (dual-axis, 280 deg, 500-2500us,
# clutch anti-stall). Spec @7.4V: max torque >=1.8 kgf*cm = 0.176 N*m, no-load
# speed <=0.05 s/60deg = 1200 deg/s, 11 g, 1:195. The clutch DISENGAGES at the
# torque limit — on the real robot exceeding this slips (grinds), not stalls.
SERVO_TORQUE = 0.176            # N*m (HPD-0127 spec, stall/clutch limit)
SERVO_KP     = 12.0

# ---- scheduler constants (verbatim GARY_v3.ino) ---------------------------------
GAITS = ["TRIPOD", "WAVE", "RIPPLE"]
PHASE = [[0,.5,0,.5,0,.5],[4/6,2/6,0/6,1/6,3/6,5/6],[0/3,1/3,2/3,0/3,1/3,2/3]]
BETA  = [0.5, 5/6, 2/3]
SPEED_CAP = [1.0, 0.15, 0.30]
MIN_CADENCE_MS = [400, 1200, 800]
WALK_SPEED_CM_S = 3.5


MESHDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes")
HAVE_MESHES = os.path.exists(os.path.join(MESHDIR, "manifest.json"))


# Real-color materials for the CAD meshes (tones straight from the
# vendor's color layers: silver brackets, black plates, white servo
# shells, copper standoffs, orange rubber feet, yellow accents).
# ALPHA IS 0 ON PURPOSE: MuJoCo rangefinders skip zero-alpha geoms
# (measured with ray_probe -- plain visual geoms ARE seen and perturbed
# the navigator), so at alpha 0 the meshes are invisible to lidar and
# massless/contactless to physics = records bit-identical BY
# CONSTRUCTION. The render shim below flips alpha to 1 only while a
# frame is actually drawn.
_MESH_TONES = {
    "m": "0.78 0.79 0.81 0", "d": "0.14 0.14 0.15 0",
    "w": "0.93 0.93 0.94 0", "c": "0.72 0.48 0.30 0",
    "o": "0.95 0.45 0.10 0", "y": "0.85 0.75 0.20 0",
}


def _mesh_xml():
    """Real CAD meshes (Bill: 'can you not make him look like he actually
    looks?'), visual-only, in each body's local frame (point-pair
    alignment in the exporter). Manifest keys are '<body>_<tone>';
    returns (asset_xml, chassis_geoms, per-leg fn)."""
    if not HAVE_MESHES:
        return "", "", lambda i, part: ""
    import json as _json
    names = sorted(_json.load(open(os.path.join(MESHDIR, "manifest.json"))))
    names = [n for n in names
             if os.path.exists(os.path.join(MESHDIR, n + ".stl"))]
    assets = "\n".join(
        '    <material name="mesh_%s" rgba="%s" specular="0.5" shininess="0.6"/>'
        % (t, rgba) for t, rgba in _MESH_TONES.items())
    assets += "\n" + "\n".join(
        '    <mesh name="%s" file="%s.stl"/>' % (n, n) for n in names)
    # THE TIBIA'S REAL SHAPE (Bill 2026-08-23: "might as well use his
    # actual geometry"): per-slab convex hulls of each leg's blade mesh,
    # as COLLISION geoms (a whole-mesh hull would fill the blade's bow).
    _colj = os.path.join(MESHDIR, "tibia_collision.json")
    _col = {}
    if os.path.exists(_colj) and os.environ.get("GARY_TIBIA_BLADE", "1") != "0":
        _col = _json.load(open(_colj))
        assets += "\n" + "\n".join(
            '    <mesh name="%s" file="%s.stl"/>' % (n, n)
            for ns in _col.values() for n in ns)

    def geoms_for(body, indent):
        out = ""
        for n in names:
            stem, _, tone = n.rpartition("_")
            if stem != body:
                continue
            out += ('\n' + ' ' * indent
                    + '<geom type="mesh" mesh="%s" material="mesh_%s"'
                      ' contype="0" conaffinity="0" mass="0" group="1"/>'
                    % (n, tone))
        return out

    # "lidar" stem = Bill's LD19 STEP (his add-on; not in the vendor
    # model), baked into base-body coords, floating at the mount until
    # he measures the riser he printed
    chassis = geoms_for("chassis", 6) + geoms_for("lidar", 6)

    def leg_mesh(i, part):
        out = geoms_for("%s%d" % (part, i), 12)
        if part == "tibia":
            for n in _col.get("tibia%d" % i, []):
                out += ('\n' + ' ' * 12
                        + '<geom type="mesh" mesh="%s" mass="0" group="3"'
                          ' material="leg_mat"/>' % n)
        return out

    return assets, chassis, leg_mesh


def mesh_show(model, on=True):
    """Flip the CAD-mesh materials between render-visible (alpha 1) and
    sensor-invisible (alpha 0). Physics/sensors must only ever run at
    alpha 0; the shim below brackets every render with this."""
    if not HAVE_MESHES:
        return
    a = 1.0 if on else 0.0
    for t in _MESH_TONES:
        mid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MATERIAL,
                                "mesh_" + t)
        if mid >= 0:
            model.mat_rgba[mid, 3] = a


def _install_mesh_shim():
    """Bracket every render with mesh_show so the meshes appear in every
    window and reel without any caller changing -- and never exist for
    the rangefinders. Renderer.update_scene bakes rgba into the scene;
    the passive viewer's sync() does the same, so flipping around those
    two calls covers nav_maze, door_gym, watch_nav and the gym reels."""
    if not HAVE_MESHES or getattr(mujoco, "_gary_mesh_shim", False):
        return
    mujoco._gary_mesh_shim = True

    _upd = mujoco.Renderer.update_scene

    def upd(self, data, *a, **kw):
        m = getattr(self, "_model", None)
        if m is not None:
            mesh_show(m, True)
        try:
            return _upd(self, data, *a, **kw)
        finally:
            if m is not None:
                mesh_show(m, False)

    mujoco.Renderer.update_scene = upd

    import mujoco.viewer as _mv
    _launch = _mv.launch_passive

    class _Handle:
        def __init__(self, h, m):
            self._h, self._m = h, m

        def sync(self, *a, **kw):
            mesh_show(self._m, True)
            try:
                return self._h.sync(*a, **kw)
            finally:
                mesh_show(self._m, False)

        def __getattr__(self, name):
            return getattr(self._h, name)

        def __enter__(self):
            self._h.__enter__()
            return self

        def __exit__(self, *a):
            return self._h.__exit__(*a)

    def launch_passive(model, data, **kw):
        return _Handle(_launch(model, data, **kw), model)

    _mv.launch_passive = launch_passive


_install_mesh_shim()


def build_xml(lift_cm=2.5, obstacles=False, bumps=False, maze=False, shortcut=False,
              corr_obs=False, rocker=False, balance=False, balance_ball=False,
              hills=False, door=False, door_gym=False, wall_gym=False,
              maze3=False):
    mesh_assets, mesh_chassis, leg_mesh = _mesh_xml()
    # with real meshes on, the collision primitives go to group 3
    # (hidden by default in renders) -- LOOK like the CAD, BEHAVE like
    # the record. Without meshes everything renders as before.
    pg = ' group="3"' if HAVE_MESHES else ''
    legs = []
    for i, ((mx, my), (rx, ry)) in enumerate(zip(MOUNTS, RADIAL)):
        yaw = math.degrees(math.atan2(ry, rx))
        legs.append(f"""
      <body name="coxa{i}" pos="{mx} {my} 0" euler="0 0 {yaw}">
        <joint name="c{i}" axis="0 0 1" range="{COXA_RANGE}" damping="0.02"/>
        <geom type="capsule" fromto="0 0 0  {L1} 0 {-FEM_DROP}" size="0.006" mass="0.030" material="joint_mat"{pg}/>{leg_mesh(i, 'coxa')}
        <body name="femur{i}" pos="{L1} 0 {-FEM_DROP}" euler="0 {-FEM0} 0">
          <joint name="f{i}" axis="0 1 0" range="{FEMUR_RANGE}" damping="0.02"/>
          <geom type="capsule" fromto="0 0 0  {L2} 0 0" size="0.007" mass="0.050" material="leg_mat"{pg}/>{leg_mesh(i, 'femur')}
          <body name="tibia{i}" pos="{L2} 0 0" euler="0 {180 - KNEE0} 0">
            <joint name="k{i}" axis="0 1 0" range="{KNEE_RANGE}" damping="0.02"/>
            <geom type="capsule" fromto="0 0 0  {L3} 0 0" size="0.005" mass="0.022" material="leg_mat"{pg}/>{leg_mesh(i, 'tibia')}
            <geom name="foot{i}" type="sphere" pos="{L3} 0 0" size="0.006" mass="0.003"
                  friction="{FOOT_MU} 0.01 0.001" material="foot_mat"{pg if os.path.exists(os.path.join(MESHDIR, f"tibia{i}_o.stl")) else ''}/>
            <!-- friction 2.0 = RUBBER CAPS (Bill: "I was thinking his feet
                 had more traction than that — they do in reality"). The
                 sim ran plastic-grade 1.2 for weeks; rubber grip changes
                 what is possible on vertical surfaces. MEASURE IN SEPT. -->
          </body>
        </body>
      </body>""")
    acts = "\n".join(
        f'    <position joint="{j}{i}" kp="{SERVO_KP}" forcerange="-{SERVO_TORQUE} {SERVO_TORQUE}" ctrlrange="{KNEE_CTRL if j == "k" else COXA_CTRL if j == "c" else FEMUR_CTRL}"/>'
        for i in range(6) for j in ("c", "f", "k"))
    rf = """    <rangefinder name="rf_ld0" site="ld0"/>
    <rangefinder name="rf_ld1" site="ld1"/>
    <rangefinder name="rf_ld2" site="ld2"/>
    <rangefinder name="rf_ld3" site="ld3"/>
    <rangefinder name="rf_ld4" site="ld4"/>
    <rangefinder name="rf_ld5" site="ld5"/>
    <rangefinder name="rf_ld6" site="ld6"/>
    <rangefinder name="rf_ld7" site="ld7"/>
    <rangefinder name="rf_ld8" site="ld8"/>
    <rangefinder name="rf_ld9" site="ld9"/>
    <rangefinder name="rf_ld10" site="ld10"/>
    <rangefinder name="rf_ld11" site="ld11"/>
    <rangefinder name="rf_ld12" site="ld12"/>
    <rangefinder name="rf_ld13" site="ld13"/>
    <rangefinder name="rf_ld14" site="ld14"/>
    <rangefinder name="rf_ld15" site="ld15"/>
    <rangefinder name="rf_ld16" site="ld16"/>
    <rangefinder name="rf_ld17" site="ld17"/>
    <rangefinder name="rf_ld18" site="ld18"/>
    <rangefinder name="rf_ld19" site="ld19"/>
    <rangefinder name="rf_ld20" site="ld20"/>
    <rangefinder name="rf_ld21" site="ld21"/>
    <rangefinder name="rf_ld22" site="ld22"/>
    <rangefinder name="rf_ld23" site="ld23"/>
    <rangefinder name="rf_nod0" site="nod0"/>
    <rangefinder name="rf_nod1" site="nod1"/>
    <rangefinder name="rf_nod2" site="nod2"/>
    <rangefinder name="rf_nod3" site="nod3"/>
    <rangefinder name="rf_nod4" site="nod4"/>
    <rangefinder name="rf_high0" site="high0"/>
    <rangefinder name="rf_high1" site="high1"/>
    <rangefinder name="rf_high2" site="high2"/>"""
    mz = """
    <!-- MAZE v2 (Bill's callout 2026-08-20: 'that's not a maze'). Fully ENCLOSED,
         uniform 0.45 m corridors, walls 0.30 tall. Path: N up the A-corridor, E over
         wall A, S down the A-B corridor, E under wall B, N up the B-C corridor, E over
         wall C, S past wall D into the CHAMBER. Only exit: the LOW TUNNEL through the
         outer east wall (lintel 0.115-0.30 = invisible to the cruising ring; visible
         crouched). Finish = beyond the tunnel, x > 2.42. -->
    <!-- outer boundary -->
    <geom name="mzOW" type="box" pos="-0.25 0.80 0.15" size="0.02 0.97 0.15" material="wall_mat"/>
    <geom name="mzOS" type="box" pos="1.03 -0.15 0.15" size="1.30 0.02 0.15" material="wall_mat"/>
    <geom name="mzON" type="box" pos="1.03 1.75 0.15" size="1.30 0.02 0.15" material="wall_mat"/>
    <geom name="mzOE1" type="box" pos="2.30 0.19 0.15" size="0.02 0.34 0.15" material="wall_mat"/>
    <geom name="mzOE2" type="box" pos="2.30 1.31 0.15" size="0.02 0.44 0.15" material="wall_mat"/>
    <geom name="mzLintel" type="box" pos="2.30 0.70 0.2075" size="0.02 0.17 0.0925" material="lintel_mat"/>
    <!-- inner walls -->
    <geom name="mzA" type="box" pos="0.20 0.575 0.15" size="0.02 0.725 0.15" material="wall_mat"/>
    <geom name="mzB" type="box" pos="0.65 1.10 0.15" size="0.02 0.65 0.15" material="wall_mat"/>
    <geom name="mzC" type="box" pos="1.10 0.575 0.15" size="0.02 0.725 0.15" material="wall_mat"/>
    <geom name="mzD1" type="box" pos="1.325 0.95 0.15" size="0.225 0.02 0.15" material="wall_mat"/>
    <geom name="mzD2" type="box" pos="2.15 0.95 0.15" size="0.15 0.02 0.15" material="wall_mat"/>
    <geom name="mzE" type="box" pos="1.55 0.325 0.15" size="0.02 0.475 0.15" material="wall_mat"/>
    """ if maze else ""
    if maze3:
        # MAZE v3 "THE VAULT" (Bill 2026-08-21: "change the layout of the
        # maze entirely, it'd be nice to see him solve a new one. add a
        # couple doors... tier 1, 2, 3 doors"). Same envelope as v2 (the
        # mind grid fits unchanged), new topology, THREE hinged doors +
        # one low crouch-tunnel as the slow bypass beside door 2 -- so a
        # locked door 2 has a second act: defeat, ban, re-search, tunnel.
        #   spawn room --D1(tier1)--> west corridor -> north corridor
        #   (pocket dead-end mid-west) -> D2(tier2) OR crouch tunnel
        #   through the same wall -> mid column -> over-wall gap ->
        #   east room --D3--> finish x>2.42
        mz = """
    <!-- outer boundary -->
    <geom name="mvOW" type="box" pos="-0.25 0.80 0.15" size="0.02 0.97 0.15" material="wall_mat"/>
    <geom name="mvOS" type="box" pos="1.03 -0.15 0.15" size="1.30 0.02 0.15" material="wall_mat"/>
    <geom name="mvON" type="box" pos="1.03 1.75 0.15" size="1.30 0.02 0.15" material="wall_mat"/>
    <geom name="mvOE1" type="box" pos="2.30 0.165 0.15" size="0.02 0.315 0.15" material="wall_mat"/>
    <geom name="mvOE2" type="box" pos="2.30 1.335 0.15" size="0.02 0.415 0.15" material="wall_mat"/>
    <!-- inner walls -->
    <geom name="mvA"  type="box" pos="0.20 0.80 0.15"  size="0.02 0.50 0.15"  material="wall_mat"/>
    <geom name="mvB"  type="box" pos="0.65 0.575 0.15" size="0.02 0.725 0.15" material="wall_mat"/>
    <geom name="mvC1" type="box" pos="1.10 0.515 0.15" size="0.02 0.185 0.15" material="wall_mat"/>
    <geom name="mvC2" type="box" pos="1.10 1.45 0.15"  size="0.02 0.30 0.15"  material="wall_mat"/>
    <geom name="mvCLintel" type="box" pos="1.10 0.11 0.2075" size="0.02 0.22 0.0925" material="lintel_mat"/>
    <geom name="mvD"  type="box" pos="1.55 0.575 0.15" size="0.02 0.275 0.15" material="wall_mat"/>
    <geom name="mvDLintel" type="box" pos="1.55 0.075 0.2075" size="0.02 0.225 0.0925" material="lintel_mat"/>
    <geom name="mvH0" type="box" pos="0.425 0.30 0.15" size="0.225 0.02 0.15" material="wall_mat"/>
    <geom name="mvH2" type="box" pos="1.325 1.30 0.15" size="0.225 0.02 0.15" material="wall_mat"/>
    <!-- the three doors: tiers are set at runtime via dof_frictionloss -->
    <!-- door panels are 0.38 long, hinged 5 mm clear of their walls:
         a 0.42 panel in the 0.41 free span was contact-SEIZED (a 2 rad/s
         hinge kick moved it 0.1 deg; a tier-1 door 'defeated' the full
         certified breach). The ~4 cm end slits double as the hinge-slit
         his ring reads for first-contact door recognition. -->
    <body name="nvdoor1" pos="-0.225 0.30 0.125">
      <joint name="nvd1_hinge" axis="0 0 1" pos="0 0 0" damping="0.15"
             range="-100 100" limited="true"/>
      <geom name="nvd1g" type="box" pos="0.19 0 0" size="0.19 0.008 0.125"
            mass="0.15" friction="1.3 0.01 0.001" material="door_mat"/>
    </body>
    <body name="nvdoor2" pos="1.10 0.705 0.125">
      <joint name="nvd2_hinge" axis="0 0 1" pos="0 0 0" damping="0.15"
             range="-100 100" limited="true"/>
      <geom name="nvd2g" type="box" pos="0 0.19 0" size="0.008 0.19 0.125"
            mass="0.15" friction="1.3 0.01 0.001" material="door_mat"/>
    </body>
    <body name="nvdoor3" pos="2.30 0.485 0.125">
      <joint name="nvd3_hinge" axis="0 0 1" pos="0 0 0" damping="0.15"
             range="-100 100" limited="true"/>
      <geom name="nvd3g" type="box" pos="0 0.19 0" size="0.008 0.19 0.125"
            mass="0.15" friction="1.3 0.01 0.001" material="door_mat"/>
    </body>
    """
    if maze and shortcut:
        # SHORTCUT + PITFALL (curriculum, Bill 2026-08-20: "shortcuts alone
        # breed corner-cutting; the pair teaches risk judgment"): an open
        # doorway through wall C at y~0.33 skips the whole B-C northern loop —
        # but its western approach is a TIPPING PLATE (hinged, tips along the
        # travel axis). Creep across and it tilts gently; charge it and it
        # slams. The pair prices the fast route.
        mz = mz.replace(
            '<geom name="mzC" type="box" pos="1.10 0.575 0.15" size="0.02 0.725 0.15" material="wall_mat"/>',
            '''<geom name="mzC1" type="box" pos="1.10 0.005 0.15" size="0.02 0.155 0.15" material="wall_mat"/>
    <geom name="mzC2" type="box" pos="1.10 0.90 0.15" size="0.02 0.40 0.15" material="wall_mat"/>
    <body name="tipplate" pos="0.94 0.33 0.015">
      <joint name="tip_hinge" axis="0 1 0" damping="0.05" range="-9 9" limited="true"/>
      <geom name="plate" type="box" size="0.08 0.12 0.0025" mass="0.25" rgba=".6 .35 .3 1"
            friction="1.2 0.01 0.001"/>
    </body>''')
    if balance_ball:
        # THE SUPREME TIER (Bill: "can he balance that platform on a ball?")
        # — free sphere under a free plate under Gary; nothing but contact
        # and friction holds the stack. The pivot point ROLLS: he must keep
        # his COM over a wandering contact, bongo-board style, with the
        # same single IMU and weight shifts. High friction = rolling, not
        # sliding; small rolling resistance for numerical sanity.
        mz = """
    <body name="bball" pos="0 0 0.09">
      <freejoint/>
      <geom name="bballg" type="sphere" size="0.09" mass="0.7"
            friction="1.6 0.02 0.002" rgba=".82 .45 .18 1"/>
    </body>
    <body name="bplate" pos="0 0 0.188">
      <freejoint/>
      <geom name="bplateg" type="box" size="0.19 0.19 0.004" mass="0.8"
            friction="1.6 0.02 0.002" material="deck_mat"/>
    </body>
    """
    if balance:
        # BALANCE GYM (Bill's curriculum, 2026-08-22: "start him on a platform
        # that gimbles... pitches and rolls in varying directions, at
        # graduating degrees, all the while Gary must maintain an upright
        # position"). A 0.38 m square plate on a 2-axis gimbal at the origin;
        # Stage A drives both hinges kinematically (mixed sines, ramping
        # amplitude); Stage B will free them — then HE balances the plate.
        mz = """
    <body name="gimbal1" pos="0 0 0.055">
      <inertial pos="0 0 0" mass="0.05" diaginertia="1e-5 1e-5 1e-5"/>
      <joint name="gx" axis="1 0 0" damping="0.6" armature="0.0002" range="-14 14" limited="true"/>
      <body name="gimbal2" pos="0 0 0">
        <joint name="gy" axis="0 1 0" damping="0.6" armature="0.0002" range="-14 14" limited="true"/>
        <geom name="balplate" type="box" size="0.19 0.19 0.004" mass="1.2"
              friction="1.3 0.01 0.001" material="deck_mat"/>
      </body>
    </body>
    """
    if maze and hills:
        # HILLS (Bill's wishlist): the B-C corridor becomes an up-and-over —
        # 8-deg ramps to a 7 cm crest. Sustained one-signed tilt = the
        # walking attitude servo's home turf (level chassis, level ring on
        # the slope). Expected discovery: the sonar floor-baseline reads an
        # upslope as a climbable bar — a real September failure mode.
        mz += """
    <geom name="rampU" type="box" pos="0.875 0.475 0.024" euler="8 0 0"
          size="0.205 0.26 0.006" friction="1.4 0.01 0.001" rgba=".5 .6 .45 1"/>
    <geom name="rampT" type="box" pos="0.875 0.80 0.066" size="0.205 0.09 0.006"
          friction="1.4 0.01 0.001" rgba=".55 .65 .5 1"/>
    <geom name="rampD" type="box" pos="0.875 1.125 0.024" euler="-8 0 0"
          size="0.205 0.26 0.006" friction="1.4 0.01 0.001" rgba=".5 .6 .45 1"/>
    """
    if wall_gym:
        # CLIMBING WALL (Bill: "maybe have him practice on a solid wall, so
        # we can get the UP-the-wall part right first") — with RUNGS: his
        # spherical feet cannot grip a smooth vertical face (friction-only,
        # they slide — the "praying" collapse), but ledges give true
        # vertical support. Get the climb right here; the smooth door
        # inherits the motion shape with pressing instead of standing.
        mz = f"""
    <geom name="cwall" type="box" pos="0 0.44 0.20" size="0.30 0.02 0.20"
          material="wall_mat"/>
    <!-- HOOK-PROFILE RUNGS (measured 2026-08-22, the hang experiment):
         a spherical rubber foot cannot hold a thin LEDGE -- both feet
         slipped together at 0.85 s no matter what. Sweep verdict: rung
         DEPTH does nothing (10 vs 40 mm: identical), rung FRICTION does
         nothing (1.4 vs 3.0: identical), rung THICKNESS is everything
         (12 -> 24 mm: hold 0.86 -> 2.54 s, mast peak 16.7 -> 18.6 cm).
         The foot needs a FACE to brace against, not a lip to balance on.
         CORRECTED 2026-08-22 by a full sweep (the first experiment
         changed only rung2 and left the other three thin -- a different
         world, and it gave a wrong answer). Sweeping ALL FOUR rungs
         across depth x thickness:
           depth  10mm -> 0.85 s   16mm -> 1.12 s   24mm -> 1.12 s
                  32mm -> 0.58 s   44mm -> 0.00 s (falls at once)
           thickness: NO measurable effect at any depth.
         So DEPTH is the lever, not thickness, and it has an OPTIMUM --
         too deep is worse than too thin, because a deep rung fouls the
         legs on the way past. Set to 24 x 24 mm (the flat top of the
         useful band, and a size that exists as real stock).
         HONEST SIZE OF THE PRIZE: 0.85 -> 1.12 s of hold, +32%. This is
         a real improvement but it is NOT the unlock -- he still cannot
         hang indefinitely on two feet with a mast-heavy nose.
         SEPTEMBER: ~24 mm square bar or dowel; deeper is not better. -->
    <geom name="rung1_lip" type="box" pos="0 {LIP_Y} 0.0565"
          size="0.30 0.0035 0.0035" friction="1.4 0.01 0.001" material="rung_mat"/>
    <geom name="rung1" type="box" pos="0 {RUNG_YC} 0.045" size="0.30 {RUNG_YH} 0.0080"
          friction="1.4 0.01 0.001" material="rung_mat"/>
    <geom name="rung2_lip" type="box" pos="0 {LIP_Y} 0.1165"
          size="0.30 0.0035 0.0035" friction="1.4 0.01 0.001" material="rung_mat"/>
    <geom name="rung2" type="box" pos="0 {RUNG_YC} 0.105" size="0.30 {RUNG_YH} 0.0080"
          friction="1.4 0.01 0.001" material="rung_mat"/>
    <geom name="rung3_lip" type="box" pos="0 {LIP_Y} 0.1765"
          size="0.30 0.0035 0.0035" friction="1.4 0.01 0.001" material="rung_mat"/>
    <geom name="rung3" type="box" pos="0 {RUNG_YC} 0.165" size="0.30 {RUNG_YH} 0.0080"
          friction="1.4 0.01 0.001" material="rung_mat"/>
    <geom name="rung4_lip" type="box" pos="0 {LIP_Y} 0.2365"
          size="0.30 0.0035 0.0035" friction="1.4 0.01 0.001" material="rung_mat"/>
    <geom name="rung4" type="box" pos="0 {RUNG_YC} 0.225" size="0.30 {RUNG_YH} 0.0080"
          friction="1.4 0.01 0.001" material="rung_mat"/>
    """
    if door_gym:
        # DOOR GYM (Bill: "have him practice the door-kick/rear-up press,
        # pop... so when he finally gets to the door, he's practiced before
        # and knows exactly how to open doors") — one door, no maze, reps.
        mz = """
    <body name="gymdoor" pos="-0.21 0.42 0.125">
      <joint name="gymdoor_hinge" axis="0 0 1" pos="0 0 0" damping="0.15"
             range="-100 100" limited="true"/>
      <geom name="gymdoorg" type="box" pos="0.21 0 0" size="0.21 0.008 0.125"
            mass="0.15" friction="1.3 0.01 0.001" material="door_mat"/>
    </body>
    """
    if maze and door:
        # DOOR (Bill's wishlist): a light hinged panel across the chamber
        # entrance (the D1-D2 gap). Ring-visible, no spring — swings where
        # pushed and stays. Nothing in his world has ever yielded to force;
        # the honest baseline should read it as a wall and despair.
        mz += """
    <body name="doorpanel" pos="1.56 0.95 0.125">
      <joint name="door_hinge" axis="0 0 1" pos="0 0 0" damping="0.15"
             range="-100 100" limited="true"/>
      <geom name="doorg" type="box" pos="0.21 0 0" size="0.21 0.008 0.125"
            mass="0.15" friction="1.3 0.01 0.001" material="door_mat"/>
    </body>
    """
    if maze and rocker:
        # MOVING-SURFACE STAGE (curriculum, Bill 2026-08-22): a rocking deck
        # spanning the FULL A-corridor width (Bill: "wide enough for him to
        # fit on... as wide as the hallway — don't know what it teaches him
        # if he continuously falls off one side"; his stance is ~0.30 m,
        # the first board was 0.19). Hinged ALONG the travel axis, DRIVEN
        # +-3.2 deg at 0.35 Hz (edge swing 11 mm — amplitude is capped by
        # the wide deck's edges reaching the floor, not by mercy). Deck top
        # 12 mm = crossable at gait lift; below the ring plane, so only the
        # IMU knows the world is moving. Norway-floor proxy.
        mz += """
    <body name="rockdeck" pos="-0.025 0.70 0.009">
      <joint name="rock_hinge" axis="0 1 0" damping="0.2" range="-5 5" limited="true"/>
      <geom name="deck" type="box" size="0.195 0.25 0.003" mass="0.8"
            friction="1.2 0.01 0.001" rgba=".55 .45 .65 1"/>
    </body>
    """
    if maze and corr_obs:
        # curriculum: OBSTACLES INSIDE CORRIDORS — step-over bars on the route
        # (12 mm crossable at gait lift; 22 mm needs the climb posture). Both
        # below the ring plane: only the down-tilted SONAR sees them coming.
        mz += """
    <geom name="cbar1" type="box" pos="-0.025 0.80 0.006" size="0.20 0.012 0.006"
          rgba=".7 .5 .2 1" friction="1.0 0.01 0.001"/>
    <geom name="cbar2" type="box" pos="0.875 0.90 0.011" size="0.20 0.012 0.011"
          rgba=".75 .4 .2 1" friction="1.0 0.01 0.001"/>
    """
    bmp = """
    <geom name="bump0" type="box" pos="-0.180 0.350 0.0058" size="0.018 0.018 0.0058" rgba=".5 0.47 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump1" type="box" pos="-0.118 0.350 0.0022" size="0.018 0.018 0.0022" rgba=".5 0.39 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump2" type="box" pos="-0.056 0.350 0.0037" size="0.018 0.018 0.0037" rgba=".5 0.42 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump3" type="box" pos="0.006 0.350 0.0033" size="0.018 0.018 0.0033" rgba=".5 0.42 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump4" type="box" pos="0.068 0.350 0.0064" size="0.018 0.018 0.0064" rgba=".5 0.48 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump5" type="box" pos="0.130 0.350 0.0061" size="0.018 0.018 0.0061" rgba=".5 0.47 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump6" type="box" pos="0.192 0.350 0.0074" size="0.018 0.018 0.0074" rgba=".5 0.50 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump7" type="box" pos="-0.180 0.415 0.0025" size="0.018 0.018 0.0025" rgba=".5 0.40 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump8" type="box" pos="-0.118 0.415 0.0045" size="0.018 0.018 0.0045" rgba=".5 0.44 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump9" type="box" pos="-0.056 0.415 0.0022" size="0.018 0.018 0.0022" rgba=".5 0.39 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump10" type="box" pos="0.006 0.415 0.0033" size="0.018 0.018 0.0033" rgba=".5 0.42 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump11" type="box" pos="0.068 0.415 0.0050" size="0.018 0.018 0.0050" rgba=".5 0.45 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump12" type="box" pos="0.130 0.415 0.0022" size="0.018 0.018 0.0022" rgba=".5 0.39 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump13" type="box" pos="0.192 0.415 0.0032" size="0.018 0.018 0.0032" rgba=".5 0.41 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump14" type="box" pos="-0.210 0.480 0.0059" size="0.018 0.018 0.0059" rgba=".5 0.47 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump15" type="box" pos="-0.148 0.480 0.0053" size="0.018 0.018 0.0053" rgba=".5 0.46 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump16" type="box" pos="-0.086 0.480 0.0033" size="0.018 0.018 0.0033" rgba=".5 0.42 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump17" type="box" pos="-0.024 0.480 0.0055" size="0.018 0.018 0.0055" rgba=".5 0.46 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump18" type="box" pos="0.038 0.480 0.0069" size="0.018 0.018 0.0069" rgba=".5 0.49 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump19" type="box" pos="0.100 0.480 0.0020" size="0.018 0.018 0.0020" rgba=".5 0.39 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump20" type="box" pos="0.162 0.480 0.0068" size="0.018 0.018 0.0068" rgba=".5 0.49 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump21" type="box" pos="-0.210 0.545 0.0062" size="0.018 0.018 0.0062" rgba=".5 0.47 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump22" type="box" pos="-0.148 0.545 0.0040" size="0.018 0.018 0.0040" rgba=".5 0.43 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump23" type="box" pos="-0.086 0.545 0.0029" size="0.018 0.018 0.0029" rgba=".5 0.41 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump24" type="box" pos="-0.024 0.545 0.0077" size="0.018 0.018 0.0077" rgba=".5 0.50 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump25" type="box" pos="0.038 0.545 0.0040" size="0.018 0.018 0.0040" rgba=".5 0.43 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump26" type="box" pos="0.100 0.545 0.0026" size="0.018 0.018 0.0026" rgba=".5 0.40 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump27" type="box" pos="0.162 0.545 0.0026" size="0.018 0.018 0.0026" rgba=".5 0.40 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump28" type="box" pos="-0.210 0.610 0.0071" size="0.018 0.018 0.0071" rgba=".5 0.49 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump29" type="box" pos="-0.148 0.610 0.0056" size="0.018 0.018 0.0056" rgba=".5 0.46 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump30" type="box" pos="-0.086 0.610 0.0068" size="0.018 0.018 0.0068" rgba=".5 0.49 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump31" type="box" pos="-0.024 0.610 0.0064" size="0.018 0.018 0.0064" rgba=".5 0.48 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump32" type="box" pos="0.038 0.610 0.0052" size="0.018 0.018 0.0052" rgba=".5 0.45 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump33" type="box" pos="0.100 0.610 0.0078" size="0.018 0.018 0.0078" rgba=".5 0.51 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump34" type="box" pos="0.162 0.610 0.0043" size="0.018 0.018 0.0043" rgba=".5 0.44 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump35" type="box" pos="-0.180 0.675 0.0053" size="0.018 0.018 0.0053" rgba=".5 0.46 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump36" type="box" pos="-0.118 0.675 0.0070" size="0.018 0.018 0.0070" rgba=".5 0.49 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump37" type="box" pos="-0.056 0.675 0.0057" size="0.018 0.018 0.0057" rgba=".5 0.46 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump38" type="box" pos="0.006 0.675 0.0072" size="0.018 0.018 0.0072" rgba=".5 0.49 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump39" type="box" pos="0.068 0.675 0.0055" size="0.018 0.018 0.0055" rgba=".5 0.46 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump40" type="box" pos="0.130 0.675 0.0062" size="0.018 0.018 0.0062" rgba=".5 0.47 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump41" type="box" pos="0.192 0.675 0.0023" size="0.018 0.018 0.0023" rgba=".5 0.40 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump42" type="box" pos="-0.180 0.740 0.0034" size="0.018 0.018 0.0034" rgba=".5 0.42 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump43" type="box" pos="-0.118 0.740 0.0037" size="0.018 0.018 0.0037" rgba=".5 0.42 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump44" type="box" pos="-0.056 0.740 0.0025" size="0.018 0.018 0.0025" rgba=".5 0.40 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump45" type="box" pos="0.006 0.740 0.0034" size="0.018 0.018 0.0034" rgba=".5 0.42 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump46" type="box" pos="0.068 0.740 0.0026" size="0.018 0.018 0.0026" rgba=".5 0.40 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump47" type="box" pos="0.130 0.740 0.0037" size="0.018 0.018 0.0037" rgba=".5 0.42 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump48" type="box" pos="0.192 0.740 0.0058" size="0.018 0.018 0.0058" rgba=".5 0.47 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump49" type="box" pos="-0.210 0.805 0.0042" size="0.018 0.018 0.0042" rgba=".5 0.43 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump50" type="box" pos="-0.148 0.805 0.0042" size="0.018 0.018 0.0042" rgba=".5 0.43 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump51" type="box" pos="-0.086 0.805 0.0033" size="0.018 0.018 0.0033" rgba=".5 0.42 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump52" type="box" pos="-0.024 0.805 0.0036" size="0.018 0.018 0.0036" rgba=".5 0.42 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump53" type="box" pos="0.038 0.805 0.0076" size="0.018 0.018 0.0076" rgba=".5 0.50 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump54" type="box" pos="0.100 0.805 0.0059" size="0.018 0.018 0.0059" rgba=".5 0.47 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump55" type="box" pos="0.162 0.805 0.0057" size="0.018 0.018 0.0057" rgba=".5 0.46 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump56" type="box" pos="-0.210 0.870 0.0030" size="0.018 0.018 0.0030" rgba=".5 0.41 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump57" type="box" pos="-0.148 0.870 0.0064" size="0.018 0.018 0.0064" rgba=".5 0.48 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump58" type="box" pos="-0.086 0.870 0.0030" size="0.018 0.018 0.0030" rgba=".5 0.41 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump59" type="box" pos="-0.024 0.870 0.0043" size="0.018 0.018 0.0043" rgba=".5 0.44 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump60" type="box" pos="0.038 0.870 0.0079" size="0.018 0.018 0.0079" rgba=".5 0.51 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump61" type="box" pos="0.100 0.870 0.0058" size="0.018 0.018 0.0058" rgba=".5 0.47 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump62" type="box" pos="0.162 0.870 0.0053" size="0.018 0.018 0.0053" rgba=".5 0.46 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump63" type="box" pos="-0.180 0.935 0.0061" size="0.018 0.018 0.0061" rgba=".5 0.47 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump64" type="box" pos="-0.118 0.935 0.0071" size="0.018 0.018 0.0071" rgba=".5 0.49 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump65" type="box" pos="-0.056 0.935 0.0067" size="0.018 0.018 0.0067" rgba=".5 0.48 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump66" type="box" pos="0.006 0.935 0.0034" size="0.018 0.018 0.0034" rgba=".5 0.42 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump67" type="box" pos="0.068 0.935 0.0022" size="0.018 0.018 0.0022" rgba=".5 0.39 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump68" type="box" pos="0.130 0.935 0.0039" size="0.018 0.018 0.0039" rgba=".5 0.43 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump69" type="box" pos="0.192 0.935 0.0036" size="0.018 0.018 0.0036" rgba=".5 0.42 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump70" type="box" pos="-0.180 1.000 0.0033" size="0.018 0.018 0.0033" rgba=".5 0.42 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump71" type="box" pos="-0.118 1.000 0.0077" size="0.018 0.018 0.0077" rgba=".5 0.50 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump72" type="box" pos="-0.056 1.000 0.0073" size="0.018 0.018 0.0073" rgba=".5 0.50 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump73" type="box" pos="0.006 1.000 0.0039" size="0.018 0.018 0.0039" rgba=".5 0.43 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump74" type="box" pos="0.068 1.000 0.0059" size="0.018 0.018 0.0059" rgba=".5 0.47 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump75" type="box" pos="0.130 1.000 0.0044" size="0.018 0.018 0.0044" rgba=".5 0.44 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump76" type="box" pos="0.192 1.000 0.0075" size="0.018 0.018 0.0075" rgba=".5 0.50 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump77" type="box" pos="-0.210 1.065 0.0048" size="0.018 0.018 0.0048" rgba=".5 0.45 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump78" type="box" pos="-0.148 1.065 0.0036" size="0.018 0.018 0.0036" rgba=".5 0.42 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump79" type="box" pos="-0.086 1.065 0.0035" size="0.018 0.018 0.0035" rgba=".5 0.42 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump80" type="box" pos="-0.024 1.065 0.0054" size="0.018 0.018 0.0054" rgba=".5 0.46 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump81" type="box" pos="0.038 1.065 0.0036" size="0.018 0.018 0.0036" rgba=".5 0.42 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump82" type="box" pos="0.100 1.065 0.0055" size="0.018 0.018 0.0055" rgba=".5 0.46 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump83" type="box" pos="0.162 1.065 0.0074" size="0.018 0.018 0.0074" rgba=".5 0.50 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump84" type="box" pos="-0.210 1.130 0.0044" size="0.018 0.018 0.0044" rgba=".5 0.44 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump85" type="box" pos="-0.148 1.130 0.0033" size="0.018 0.018 0.0033" rgba=".5 0.42 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump86" type="box" pos="-0.086 1.130 0.0080" size="0.018 0.018 0.0080" rgba=".5 0.51 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump87" type="box" pos="-0.024 1.130 0.0051" size="0.018 0.018 0.0051" rgba=".5 0.45 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump88" type="box" pos="0.038 1.130 0.0025" size="0.018 0.018 0.0025" rgba=".5 0.40 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump89" type="box" pos="0.100 1.130 0.0023" size="0.018 0.018 0.0023" rgba=".5 0.40 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump90" type="box" pos="0.162 1.130 0.0027" size="0.018 0.018 0.0027" rgba=".5 0.40 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump91" type="box" pos="-0.180 1.195 0.0058" size="0.018 0.018 0.0058" rgba=".5 0.47 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump92" type="box" pos="-0.118 1.195 0.0068" size="0.018 0.018 0.0068" rgba=".5 0.49 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump93" type="box" pos="-0.056 1.195 0.0045" size="0.018 0.018 0.0045" rgba=".5 0.44 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump94" type="box" pos="0.006 1.195 0.0024" size="0.018 0.018 0.0024" rgba=".5 0.40 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump95" type="box" pos="0.068 1.195 0.0043" size="0.018 0.018 0.0043" rgba=".5 0.44 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump96" type="box" pos="0.130 1.195 0.0080" size="0.018 0.018 0.0080" rgba=".5 0.51 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump97" type="box" pos="0.192 1.195 0.0052" size="0.018 0.018 0.0052" rgba=".5 0.45 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump98" type="box" pos="-0.180 1.260 0.0078" size="0.018 0.018 0.0078" rgba=".5 0.51 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump99" type="box" pos="-0.118 1.260 0.0072" size="0.018 0.018 0.0072" rgba=".5 0.49 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump100" type="box" pos="-0.056 1.260 0.0021" size="0.018 0.018 0.0021" rgba=".5 0.39 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump101" type="box" pos="0.006 1.260 0.0063" size="0.018 0.018 0.0063" rgba=".5 0.48 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump102" type="box" pos="0.068 1.260 0.0061" size="0.018 0.018 0.0061" rgba=".5 0.47 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump103" type="box" pos="0.130 1.260 0.0052" size="0.018 0.018 0.0052" rgba=".5 0.45 .3 1" friction="1.2 0.01 0.001"/>
    <geom name="bump104" type="box" pos="0.192 1.260 0.0036" size="0.018 0.018 0.0036" rgba=".5 0.42 .3 1" friction="1.2 0.01 0.001"/>
    """ if bumps else ""
    obs = """
    <!-- obstacle course along +y: two step-over bars, then a crawl-under bridge -->
    <geom name="bar1"   type="box" pos="0 0.35 0.006"  size="0.20 0.012 0.006" rgba=".7 .5 .2 1" friction="1.0 0.01 0.001"/>
    <geom name="bar2"   type="box" pos="0 0.70 0.011"  size="0.20 0.012 0.011" rgba=".75 .4 .2 1" friction="1.0 0.01 0.001"/>
    <geom name="deck"   type="box" pos="0 1.15 0.131"  size="0.20 0.10 0.012" rgba=".4 .45 .6 .9"/>
    <geom name="postL"  type="box" pos="-0.19 1.15 0.0595" size="0.01 0.10 0.0595" rgba=".4 .45 .6 1"/>
    <geom name="postR"  type="box" pos=" 0.19 1.15 0.0595" size="0.01 0.10 0.0595" rgba=".4 .45 .6 1"/>
    <!-- see-saw: hinged plank, pivot 24 mm up — IMU workout (Bill, 2026-08-18) -->
    <body name="seesaw" pos="0 1.85 0.024">
      <joint name="seesaw_hinge" axis="1 0 0" damping="0.15" range="-14 14" limited="true"/>
      <geom name="plank" type="box" size="0.13 0.30 0.004" mass="0.35" rgba=".55 .4 .25 1"
            friction="1.2 0.01 0.001"/>
    </body>
    <!-- little stairs (Bill, 2026-08-18): 4 steps, 15 mm rise / 60 mm run, then a plateau -->
    <geom name="step1" type="box" pos="0 2.43 0.0075" size="0.245 0.03 0.0075" rgba=".5 .5 .55 1" friction="1.2 0.01 0.001"/>
    <geom name="step2" type="box" pos="0 2.49 0.015"  size="0.245 0.03 0.015"  rgba=".55 .55 .6 1" friction="1.2 0.01 0.001"/>
    <geom name="step3" type="box" pos="0 2.55 0.0225" size="0.245 0.03 0.0225" rgba=".6 .6 .65 1" friction="1.2 0.01 0.001"/>
    <geom name="step4" type="box" pos="0 2.61 0.030"  size="0.245 0.03 0.030"  rgba=".65 .65 .7 1" friction="1.2 0.01 0.001"/>
    <geom name="plateau" type="box" pos="0 2.79 0.030" size="0.245 0.15 0.030" rgba=".65 .65 .7 1" friction="1.2 0.01 0.001"/>
    <!-- lane rails (Bill, 2026-08-19: "he's content going around the bridge") — going
         around is out of bounds now; the rails also give the ring a corridor to center on -->
    <geom name="railL" type="box" pos="-0.26 1.20 0.06" size="0.015 1.30 0.06" rgba=".35 .32 .3 1"/>
    <geom name="railR" type="box" pos=" 0.26 1.20 0.06" size="0.015 1.30 0.06" rgba=".35 .32 .3 1"/>
    <!-- enclosure (Bill, 2026-08-19): dead end past the plateau — forces a 180 and a return run -->
    <geom name="wall_back" type="box" pos="0 3.00 0.155"    size="0.28 0.02 0.125" material="wall_mat"/>
    <geom name="wall_left" type="box" pos="-0.27 2.70 0.155" size="0.02 0.32 0.125" material="wall_mat"/>
    <geom name="wall_right" type="box" pos="0.27 2.70 0.155" size="0.02 0.32 0.125" material="wall_mat"/>
    """ if obstacles else ""

    return f"""<mujoco model="gary">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <visual>
    <global offwidth="1920" offheight="1080" elevation="-20" azimuth="130"/>
    <quality shadowsize="8192" offsamples="8"/>
    <headlight ambient="0.30 0.31 0.34" diffuse="0.42 0.42 0.45"
               specular="0.25 0.25 0.28"/>
    <map force="0.1" zfar="30" shadowclip="1.5" shadowscale="0.8"/>
    <rgba haze="0.14 0.18 0.24 1"/>
  </visual>
  <compiler meshdir="{MESHDIR}"/>
  <asset>
{mesh_assets}
    <texture name="sky" type="skybox" builtin="gradient"
             rgb1="0.22 0.30 0.42" rgb2="0.03 0.04 0.06"
             width="512" height="3072"/>
    <texture name="checker" type="2d" builtin="checker"
             rgb1="0.17 0.19 0.21" rgb2="0.26 0.29 0.32"
             markrgb="0.4 0.42 0.45" mark="edge" width="512" height="512"/>
    <material name="floor_mat" texture="checker" texrepeat="6 6"
              texuniform="true" reflectance="0.22" specular="0.45"
              shininess="0.35"/>
    <material name="chassis_mat" rgba="0.11 0.115 0.135 1" specular="0.85"
              shininess="0.82" reflectance="0.14"/>
    <material name="leg_mat" rgba="0.58 0.60 0.65 1" specular="0.75"
              shininess="0.70" reflectance="0.10"/>
    <material name="joint_mat" rgba="0.30 0.32 0.36 1" specular="0.6"
              shininess="0.5"/>
    <material name="foot_mat" rgba="0.88 0.36 0.10 1" specular="0.25"
              shininess="0.15"/>
    <material name="lidar_mat" rgba="0.10 0.34 0.78 1" specular="1.0"
              shininess="0.95" reflectance="0.30"/>
    <material name="wall_mat" rgba="0.33 0.34 0.39 1" specular="0.30"
              shininess="0.25" reflectance="0.06"/>
    <material name="lintel_mat" rgba="0.46 0.36 0.30 1" specular="0.25"
              shininess="0.2"/>
    <material name="door_mat" rgba="0.66 0.44 0.22 1" specular="0.35"
              shininess="0.3" reflectance="0.05"/>
    <material name="rung_mat" rgba="0.78 0.62 0.24 1" specular="0.7"
              shininess="0.6" reflectance="0.12"/>
    <material name="deck_mat" rgba="0.45 0.50 0.58 1" specular="0.5"
              shininess="0.4" reflectance="0.10"/>
  </asset>
  <worldbody>
    <light name="key" pos="1.2 -0.9 2.2" dir="-0.45 0.35 -1"
           directional="true" castshadow="true"
           diffuse="0.62 0.62 0.60" specular="0.30 0.30 0.30"/>
    <light name="fill" pos="-1.6 1.4 1.6" dir="0.5 -0.45 -1"
           directional="true" castshadow="false"
           diffuse="0.22 0.24 0.30" specular="0.05 0.05 0.05"/>
    <geom name="floor" type="plane" size="3 3 .1" material="floor_mat"
          friction="1.2 0.01 0.001"/>
    {obs}{bmp}{mz}
    <body name="base" pos="0 0 {COXA_Z}">
      <freejoint/>
      <geom type="box" size="0.055 0.075 0.014" mass="0.62" material="chassis_mat"{pg}/>
{mesh_chassis}
      <!-- D500 LiDAR, 46 g (confirmed): second tier, centerline, front face at the
           body's front edge, one body-thickness above the chassis box (Bill, 2026-08-18) -->
      <geom name="lidar" type="cylinder" size="0.0193 0.0168" pos="0 0.0555 0.0588"
            mass="0.046" material="lidar_mat"{' group="3"' if os.path.exists(os.path.join(MESHDIR, "lidar_d.stl")) else ''}/>
      <site name="nose" pos="0 0.075 0.02" size="0.004" rgba="1 0 0 1"/>
      <!-- sensor suite (2026-08-18): sonar = the ultrasonic eyes (front, 10 deg down-tilt so
           low bars break the floor-baseline); ld0-23 = D500 planar ring at mount height
           (CANNOT see the bars - too low; sees walls/posts); nod0-4 = nod-scan sample
           (front sector tilted +12 deg - what catches the bridge deck, like NODSCAN pass 1) -->
      <site name="sonar_site" pos="0 0.076 -0.010" zaxis="0 0.9848 -0.1736" size="0.002" rgba="0 1 0 1"/>
      <site name="ld0" pos="0.0000 0.0765 0.0588" zaxis="0.0000 1.0000 0" size="0.002"/>
      <site name="ld1" pos="0.0054 0.0758 0.0588" zaxis="0.2588 0.9659 0" size="0.002"/>
      <site name="ld2" pos="0.0105 0.0737 0.0588" zaxis="0.5000 0.8660 0" size="0.002"/>
      <site name="ld3" pos="0.0148 0.0703 0.0588" zaxis="0.7071 0.7071 0" size="0.002"/>
      <site name="ld4" pos="0.0182 0.0660 0.0588" zaxis="0.8660 0.5000 0" size="0.002"/>
      <site name="ld5" pos="0.0203 0.0609 0.0588" zaxis="0.9659 0.2588 0" size="0.002"/>
      <site name="ld6" pos="0.0210 0.0555 0.0588" zaxis="1.0000 0.0000 0" size="0.002"/>
      <site name="ld7" pos="0.0203 0.0501 0.0588" zaxis="0.9659 -0.2588 0" size="0.002"/>
      <site name="ld8" pos="0.0182 0.0450 0.0588" zaxis="0.8660 -0.5000 0" size="0.002"/>
      <site name="ld9" pos="0.0148 0.0407 0.0588" zaxis="0.7071 -0.7071 0" size="0.002"/>
      <site name="ld10" pos="0.0105 0.0373 0.0588" zaxis="0.5000 -0.8660 0" size="0.002"/>
      <site name="ld11" pos="0.0054 0.0352 0.0588" zaxis="0.2588 -0.9659 0" size="0.002"/>
      <site name="ld12" pos="0.0000 0.0345 0.0588" zaxis="0.0000 -1.0000 0" size="0.002"/>
      <site name="ld13" pos="-0.0054 0.0352 0.0588" zaxis="-0.2588 -0.9659 0" size="0.002"/>
      <site name="ld14" pos="-0.0105 0.0373 0.0588" zaxis="-0.5000 -0.8660 0" size="0.002"/>
      <site name="ld15" pos="-0.0148 0.0407 0.0588" zaxis="-0.7071 -0.7071 0" size="0.002"/>
      <site name="ld16" pos="-0.0182 0.0450 0.0588" zaxis="-0.8660 -0.5000 0" size="0.002"/>
      <site name="ld17" pos="-0.0203 0.0501 0.0588" zaxis="-0.9659 -0.2588 0" size="0.002"/>
      <site name="ld18" pos="-0.0210 0.0555 0.0588" zaxis="-1.0000 -0.0000 0" size="0.002"/>
      <site name="ld19" pos="-0.0203 0.0609 0.0588" zaxis="-0.9659 0.2588 0" size="0.002"/>
      <site name="ld20" pos="-0.0182 0.0660 0.0588" zaxis="-0.8660 0.5000 0" size="0.002"/>
      <site name="ld21" pos="-0.0148 0.0703 0.0588" zaxis="-0.7071 0.7071 0" size="0.002"/>
      <site name="ld22" pos="-0.0105 0.0737 0.0588" zaxis="-0.5000 0.8660 0" size="0.002"/>
      <site name="ld23" pos="-0.0054 0.0758 0.0588" zaxis="-0.2588 0.9659 0" size="0.002"/>
      <site name="nod0" pos="-0.0105 0.0737 0.0620" zaxis="-0.4891 0.8471 0.2079" size="0.002"/>
      <site name="nod1" pos="-0.0054 0.0758 0.0620" zaxis="-0.2532 0.9448 0.2079" size="0.002"/>
      <site name="nod2" pos="0.0000 0.0765 0.0620" zaxis="0.0000 0.9781 0.2079" size="0.002"/>
      <site name="nod3" pos="0.0054 0.0758 0.0620" zaxis="0.2532 0.9448 0.2079" size="0.002"/>
      <site name="nod4" pos="0.0105 0.0737 0.0620" zaxis="0.4891 0.8471 0.2079" size="0.002"/>
      <site name="high0" pos="-0.0072 0.0752 0.0640" zaxis="-0.2802 0.7698 0.5736" size="0.002"/>
      <site name="high1" pos="0.0000 0.0765 0.0640" zaxis="0.0000 0.8192 0.5736" size="0.002"/>
      <site name="high2" pos="0.0072 0.0752 0.0640" zaxis="0.2802 0.7698 0.5736" size="0.002"/>
{''.join(legs)}
    </body>
  </worldbody>
  <actuator>
{acts}
  </actuator>
  <sensor>
    <rangefinder name="sonar" site="sonar_site"/>
{rf}
  </sensor>
</mujoco>"""


def leg_ik(out_m, down_m):
    """foot in leg plane -> (femur_pitch_deg, knee_interior_deg) or None."""
    r = out_m - L1
    down = down_m - FEM_DROP * 0          # down is measured from the femur axis already
    D2 = r * r + down * down
    D = math.sqrt(D2)
    if D > L2 + L3 - 1e-6 or D < abs(L2 - L3) + 1e-6:
        return None
    knee = math.degrees(math.acos((L2 * L2 + L3 * L3 - D2) / (2 * L2 * L3)))
    fem = math.degrees(math.atan2(-down, r) + math.acos((L2 * L2 + D2 - L3 * L3) / (2 * L2 * D)))
    # CONTROLLER knee clamp (2026-08-22). Every record before today was
    # set with the knee joint STOPPED at KNEE0+45 = 101.7 deg: the gaits
    # command past it and MuJoCo's joint limit -- a free, infinitely
    # strong brace -- held the leg there. The real knee has no stop
    # (Bill: straightens, even inverts); its servo must hold the pose
    # alone at 0.176 N*m, which is the stall that killed the July knees.
    # With the joint freed, this clamp is the honest version of the
    # old behaviour: the command stops at the same angle and the SERVO
    # carries it. Ports to firmware as-is (knee-guard analog).
    # GARY_KNEE_CMD_MAX=180 lifts it for straight-leg experiments.
    if knee > KNEE_CMD_MAX:
        knee = KNEE_CMD_MAX
    return fem, knee


class GaitDriver:
    """Port of run_custom_gait_scheduler: body-frame foot targets -> per-leg (coxa, femur, knee)."""

    def __init__(self, gait="WAVE", stick=1.0, lift_cm=2.5, cadence_ms=None, speed_cap=None,
                 touchdown="cosine"):
        self.g = GAITS.index(gait)
        cap = SPEED_CAP[self.g] if speed_cap is None else speed_cap
        self.v = WALK_SPEED_CM_S * stick * cap * 0.01          # m/s forward (+y)
        self.T = (MIN_CADENCE_MS[self.g] if cadence_ms is None else cadence_ms) / 1000.0
        self.amp = self.v * self.T / 2.0                        # m half-stroke (vendor convention)
        self.beta = BETA[self.g]
        self.lift = lift_cm * 0.01
        self.touchdown = touchdown   # "cosine" = current firmware; "match" = velocity-matched Hermite
        self.omega = 0.0             # rad/s yaw, live-settable (like the firmware's omega amp)
        self.vx = 0.0                # m/s strafe (+x = right), live-settable — omnidirectional
        self.height = 0.0            # m body-height offset: + = stand taller, - = crouch
        self.body_pitch = 0.0        # rad, + = nose up   (gimbal posing, like NODSCAN)
        self.body_roll = 0.0         # rad, + = right side down
        self.body_pivot_y = 0.0      # pitch pivot (see tilt calc)
        self.leg_h = [0.0] * 6       # per-leg height offset (Bill's
        # WEDGE: rears crouched, mids tall -> the STANCE angles the
        # body nose-up like a ramp; then the gait walks him forward
        # in that posture. Stance geometry beats commanded rotation:
        # no loaded/unloaded confusion, just legs of different
        # lengths, walking.)
        self.body_dx = 0.0           # m, body-frame COM shift (+ = right):
        self.leg_lift = [0.0] * 6    # PER-LEG extra swing lift (the
        # stair gait's fronts must clear the next rung's lip -- Bill:
        # "he's gotta be able to raise those front legs up higher").
        # Zero default = every record's trajectories bit-identical.
        self.leg_dy = [0.0] * 6      # PER-LEG fore/aft offset. body_dy
        #     shifts the WHOLE stance, which cannot crawl a body toward
        #     a wall one leg at a time -- and the walking gait lifts legs
        #     in TRIPOD PAIRS, which is precisely the support break Bill
        #     forbids while two feet are anchored on a wall ("moving the
        #     back two feet at the same time while only his middles are
        #     on the ground... surely produces a fall"). This lets ONE
        #     leg step closer while the other five hold.
        self.body_dy = 0.0           # feet move opposite; balance-gym stage B
        self._press_t = 0.0          # climb-press stager
        self.press_hi = 0.0          # extra reach height (rear-mount)
        self.foot_lock = None        # {leg: (coxa, out, down)} — exact
        # leg-frame commands computed by a caller who KNOWS the world
        # (true world-anchoring: world target -> live body pose ->
        # leg frame, every tick; plant_h-style height patches cannot
        # compensate dy sweeps, pitch, or wobble)
        self.tpc = None              # GEN-3 THREE-POINT CLIMB (Bill's
        # law: 'step them up one at a time' — never commit both fronts
        # to the wall until it truly bears load). dict per front leg:
        # {'rung': kdz, 'plant_h': height-at-plant} or None (ground).
        # WORLD-ANCHORED HOLDS — the night's buried bug: holds were
        # body-frame, so every body motion DRAGGED the planted feet
        # off the rungs. Now the hold target compensates for body
        # height changes since plant: the foot stays, the body moves
        # around it, like every climber since the first gecko.
        self.press_climb = 0.0       # 0..1: the fronts WALK UP the
        # panel as the body rises — a fixed plant height PINS the
        # nose down (position servos hold the feet, so the body
        # cannot pitch: Bill saw 'never upright, everything flat')
        self.kick_leg = None         # fire a one-leg STRIKE (Bill: door kick)
        self.ram_full = False        # THE YAW-POWERED RAM (Bill: "the
        #     YAW! is what will make that battering ram action really
        #     shine... chassis positioned fully back, and then thrown
        #     forward, with legs planted"): when a leg's IK fails during
        #     a body shift, sweep THAT hip to its own envelope instead of
        #     freezing -- mids whip full-stroke, diagonals give their max
        self.body_yaw = 0.0          # yaw POSE (rad): true rotation of all
        #     foot targets about the body centre -- with feet planted the
        #     BODY yaws, full amplitude, no skate. The missing 6th DoF
        #     (pitch, roll, dx, dy existed; yaw never did). Powers the
        #     shame-shake wag and Bill's cornering yaw lead.
        self.coxa_bias = 0.0         # uniform coxa offset, all stance legs:
        #     with feet PLANTED the body counter-yaws — the shake-it-off
        #     wiggle (Bill on the shame shake: omega stepping "is wrong,
        #     should be similar to one of his actions already scripted" —
        #     this is the factory SHAKE, not a walking turn)
        self._kick_t = 0.0           # cock 0.25s -> strike 0.11s -> retract
        self.press = 0               # REAR-UP PRESS (Bill): 0 = off.
        # 1 = fronts on the door, gait drives on the back four (mobile push).
        # 2 = fronts AND middles braced against the surface (Bill: "front
        #     and middle should be able to lean against the door/wall") —
        #     middles swing their coxa ~57 deg forward into an angled brace;
        #     only the rear pair drives. Maximum contact, the power shove.
        self._last = np.zeros(18)    # last commanded targets (held on per-leg IK failure)
        self.t = 0.0

    def ctrl(self, dt):
        self.t += dt
        cyc = (self.t / self.T) % 1.0
        ampy = self.v * self.T / 2.0              # recompute so live speed changes take effect
        ampx = self.vx * self.T / 2.0             # strafe half-stroke
        self.amp = ampy                            # kept for older callers/metrics
        om_amp = self.omega * self.T / 2.0        # rad half-stroke for yaw
        # graceful yaw saturation: instead of clamping each leg's contribution
        # (which distorts the rotation field and collapses turning), scale the
        # WHOLE yaw field down so the largest leg stroke stays within 3 cm.
        max_lever = 0.117                          # largest |r_foot| over the six legs
        # IN-PLACE spins get the full stroke (Bill: "he really should be
        # turning-in-place a lot faster") -- the 3 cm cap is the April
        # firmware's walking-turn conservatism; a pure spin has no
        # translation stroke competing for the envelope. September note:
        # raise OMEGA_AMP_CLAMP for v==0 turns in firmware too.
        _tc = 0.03 if abs(self.v) > 0.005 else 0.055
        if abs(om_amp) * max_lever > _tc:
            om_amp = math.copysign(_tc / max_lever, om_amp)
        out = np.zeros(18)
        if self.press != 3:
            self._press_t = 0.0
        if self.kick_leg is not None:
            self._kick_t += dt
            if self._kick_t > 0.80:
                self.kick_leg, self._kick_t = None, 0.0
        for i in range(6):
            if os.environ.get("GARY_IK_PROBE") and i == 1:
                self._pbn = getattr(self, "_pbn", 0) + 1
                if self._pbn % 400 == 0:
                    import sys as _sys
                    print("BRANCH leg1: fl=%s tpc=%s kick=%s"
                          % (self.foot_lock is not None
                             and 1 in self.foot_lock,
                             self.tpc is not None and 1 in (self.tpc or {}),
                             self.kick_leg), file=_sys.stderr)
            if self.foot_lock is not None and i in self.foot_lock:
                _kc, _ko, _kd = self.foot_lock[i]
                ik = leg_ik(_ko, _kd)
                if ik is not None:
                    fem, knee = ik
                    out[3*i + 0] = _kc
                    out[3*i + 1] = -math.radians(fem - FEM0)
                    out[3*i + 2] = -math.radians(knee - KNEE0)
                else:
                    out[3*i:3*i+3] = self._last[3*i:3*i+3]
                continue
            if self.tpc is not None and i in (1, 2):
                st3 = self.tpc.get(i)
                if st3 is None:
                    pass                      # this front has GROUND DUTY:
                    # fall through to ordinary gait — the three-point law
                elif st3.get("arc") is not None:
                    # stepping arc to a rung: in, up past, out, down-on
                    a3 = st3["arc"]
                    pr3 = min(1.0, a3["pr"])
                    ka_v, kb_v = a3["from"], a3["to"]
                    if pr3 < 0.40:
                        _s = pr3 / 0.40
                        kdz = ka_v + (kb_v + 0.018 - ka_v) * _s
                        arc_in = 0.045 * _s
                    elif pr3 < 0.70:
                        _s = (pr3 - 0.40) / 0.30
                        kdz = kb_v + 0.018
                        arc_in = 0.045 * (1.0 - _s)
                    else:
                        _s = (pr3 - 0.70) / 0.30
                        kdz = kb_v + 0.018 * (1.0 - _s)
                        arc_in = 0.0
                    _down = NEUTRAL_DOWN + self.height - kdz
                    _rmax = math.sqrt(max(1e-6, 0.112**2 - _down * _down))
                    kout = L1 + _rmax - 0.002 - NEUTRAL_OUT - arc_in
                    kcoxa = 0.785 if i == 1 else -0.785
                    ik = leg_ik(NEUTRAL_OUT + kout,
                                NEUTRAL_DOWN + self.height - kdz)
                    if ik is not None:
                        fem, knee = ik
                        out[3*i + 0] = kcoxa
                        out[3*i + 1] = -math.radians(fem - FEM0)
                        out[3*i + 2] = -math.radians(knee - KNEE0)
                    else:
                        out[3*i:3*i+3] = self._last[3*i:3*i+3]
                    continue
                else:
                    # HOLDING a rung, WORLD-ANCHORED: compensate for body
                    # height change since plant so the foot stays put in
                    # the world while the body climbs past it
                    kdz = st3["rung"] - (self.height - st3["plant_h"])
                    _down = NEUTRAL_DOWN + self.height - kdz
                    _rmax = math.sqrt(max(1e-6, 0.112**2 - _down * _down))
                    kout = L1 + _rmax + 0.002 - NEUTRAL_OUT   # pressed IN
                    kcoxa = 0.785 if i == 1 else -0.785
                    ik = leg_ik(NEUTRAL_OUT + kout,
                                NEUTRAL_DOWN + self.height - kdz)
                    if ik is not None:
                        fem, knee = ik
                        out[3*i + 0] = kcoxa
                        out[3*i + 1] = -math.radians(fem - FEM0)
                        out[3*i + 2] = -math.radians(knee - KNEE0)
                    else:
                        out[3*i:3*i+3] = self._last[3*i:3*i+3]
                    continue
            if self.press == 3 and i in (1, 2):
                # WALL-CLIMB GAIT (Bill: "his front feet would be moving up
                # the wall while his back feet were moving in towards it,
                # effectively climbing the wall... get him up on the wall
                # first"): the fronts STEP up the panel rung by rung —
                # plant both at the base rung, then alternate upward steps,
                # each step a small lift-off arc to the next height. The
                # reach for every height follows the leg envelope, so feet
                # pull IN as they rise and the body must close and angle:
                # the climb posture EMERGES from geometry. The rear four
                # keep ordinary gait (caller drives v>0) and walk in
                # beneath him. Speed and force are the next generation.
                self._press_t += dt / 2.0        # (two legs per pass)
                pt = self._press_t
                RUNGS = (0.046, 0.091, 0.136, 0.157, 0.157)
                # = ledge tops (4/8.5/13 cm + cap): feet land ON the
                # rungs of the practice wall; kdz ~ foot height

                def rung_state(leg, tt):
                    # initial staged plant: leg1 0-0.5s, leg2 0.5-1.0s
                    if tt < 1.0:
                        r0 = min(1.0, tt / 0.5) if leg == 1 else                              max(0.0, min(1.0, (tt - 0.5) / 0.5))
                        return 0, 0, r0          # rising to rung 0
                    cyc, ph = divmod(tt - 1.0, 2.4)
                    k = min(int(cyc), len(RUNGS) - 2)
                    # STEP, don't throw (Bill: "he probably doesn't have to
                    # throw his front legs up, he could just step them up
                    # one at a time") — slow deliberate 1.0 s steps produce
                    # no recoil to counter; the pounce is a later lesson
                    if leg == 1 and ph < 1.0:
                        return k, k + 1, ph / 1.0
                    if leg == 2 and 1.1 <= ph < 2.1:
                        return k, k + 1, (ph - 1.1) / 1.0
                    # holding: leg2 lags one rung mid-cycle
                    hold = k + 1 if (leg == 1 or ph >= 2.1) else k
                    if leg == 2 and ph < 1.1:
                        hold = k
                    return min(hold, len(RUNGS) - 1), min(hold, len(RUNGS) - 1), 1.0

                ka, kb, pr = rung_state(i, pt)
                kdz = RUNGS[ka] + (RUNGS[kb] - RUNGS[ka]) * pr
                if pt < 1.0:
                    kdz = 0.028 + (RUNGS[0] - 0.028) * pr
                _down = NEUTRAL_DOWN + self.height - kdz
                _rmax = math.sqrt(max(1e-6, 0.112**2 - _down * _down))
                kout = L1 + _rmax - 0.004 - NEUTRAL_OUT
                if pr < 1.0:
                    kout -= 0.012 * math.sin(math.pi * pr)   # lift-off arc
                kcoxa = 0.785 if i == 1 else -0.785
                ik = leg_ik(NEUTRAL_OUT + kout,
                            NEUTRAL_DOWN + self.height - kdz)
                if ik is not None:
                    fem, knee = ik
                    out[3*i + 0] = kcoxa
                    out[3*i + 1] = -math.radians(fem - FEM0)
                    out[3*i + 2] = -math.radians(knee - KNEE0)
                else:
                    out[3*i:3*i+3] = self._last[3*i:3*i+3]
                continue
            if self.press == 4 and i in (4, 5):
                # REVERSED press (Bill: "he could do a 180 on his 3rd
                # attempt, and perform his forward door-attack, backwards
                # ... his LiDAR would help counterweight him"): the REARS
                # aim STRAIGHT BACK onto the panel behind. Aim computed
                # from the mount geometry, not guessed (convention law).
                rx, ry = RADIAL[i]
                kcoxa = math.atan2(-1.0, 0.0) - math.atan2(ry, rx)
                while kcoxa > math.pi: kcoxa -= 2 * math.pi
                while kcoxa < -math.pi: kcoxa += 2 * math.pi
                kout, kdz = 0.062, 0.055 + self.press_hi
                ik = leg_ik(NEUTRAL_OUT + kout,
                            NEUTRAL_DOWN + self.height - kdz)
                if ik is not None:
                    fem, knee = ik
                    out[3*i + 0] = kcoxa
                    out[3*i + 1] = -math.radians(fem - FEM0)
                    out[3*i + 2] = -math.radians(knee - KNEE0)
                else:
                    out[3*i:3*i+3] = self._last[3*i:3*i+3]
                continue
            if self.press in (1, 2) and (i in (1, 2)
                               or (self.press >= 2 and i in (0, 3))):
                if i in (1, 2):
                    # press_hi lifts the reach pose (rear-mount walk:
                    # low-hanging fronts caught the ground in transit
                    # and re-loaded — the collapse's last hiding spot)
                    # fronts aim STRAIGHT AHEAD onto the panel (their mounts
                    # point 45 deg out — radial extension just hovered them
                    # diagonally: Bill saw "scooting his fronts", no press),
                    # and press HIGH on the panel for hinge torque
                    kcoxa = 0.785 if i == 1 else -0.785
                    kout, kdz = 0.062, 0.055 + self.press_hi
                else:
                    # middles: coxa swung hard forward, angled brace
                    kcoxa = 1.0 if i == 0 else -1.0
                    kout, kdz = 0.030, 0.020
                ik = leg_ik(NEUTRAL_OUT + kout,
                            NEUTRAL_DOWN + self.height - kdz)
                if ik is not None:
                    fem, knee = ik
                    out[3*i + 0] = kcoxa
                    out[3*i + 1] = -math.radians(fem - FEM0)
                    out[3*i + 2] = -math.radians(knee - KNEE0)
                else:
                    out[3*i:3*i+3] = self._last[3*i:3*i+3]
                continue
            if i == self.kick_leg:
                # THE KICK (Bill: "style points... tries to kick it
                # open first"): cock the leg up and back, DRIVE it
                # forward-out at door height, retract. The other five
                # hold stance (caller sets v=0) — the balance work
                # made a five-leg stand free.
                kt = self._kick_t
                if kt < 0.22:                    # cock
                    kr, kz = -0.030 * (kt / 0.22), 0.052 * (kt / 0.22)
                elif kt < 0.30:                  # STRIKE - fast, WITHIN reach
                    ks = (kt - 0.22) / 0.08
                    kr, kz = -0.030 + 0.095 * ks, 0.052 - 0.006 * ks
                    # anatomical cap: out beyond ~0.14 total fails leg_ik and
                    # freezes the foot (his arms are shorter than the ambition)
                elif kt < 0.52:                  # FOLLOW-THROUGH hold: the
                    kr, kz = 0.065, 0.046        # servo needs ~0.2 s to arrive
                    # - the un-held strike peaked 1.3 cm short, DURING the
                    # commanded retract
                else:                            # retract
                    ks = min(1.0, (kt - 0.52) / 0.26)
                    kr, kz = 0.065 * (1 - ks), 0.046 * (1 - ks)
                ik = leg_ik(NEUTRAL_OUT + kr,
                            NEUTRAL_DOWN + self.height - kz)
                if ik is not None:
                    fem, knee = ik
                    # aim FORWARD, not down the 45-deg mount diagonal —
                    # the same aim bug the press had; the first gym's kicks
                    # struck air 45 deg off-target
                    out[3*i + 0] = 0.785 if i == 1 else (-0.785 if i == 2 else 0.0)
                    out[3*i + 1] = -math.radians(fem - FEM0)
                    out[3*i + 2] = -math.radians(knee - KNEE0)
                else:
                    out[3*i:3*i+3] = self._last[3*i:3*i+3]
                continue
            ph = (cyc - PHASE[self.g][i]) % 1.0
            # normalized fore-aft profile kprof in [-1, +1]; dz = lift
            if ph < self.beta:
                kprof = 1.0 - 2.0 * ph / self.beta
                dz = 0.0
            else:
                st = (ph - self.beta) / (1.0 - self.beta)
                if self.touchdown == "match":
                    # velocity-matched swing: cubic Hermite -1 -> +1, endpoint slope
                    # equal to the stance slope -> zero touchdown scoot by construction.
                    m = -2.0 * (1.0 - self.beta) / self.beta
                    h00 = 2*st**3 - 3*st**2 + 1; h10 = st**3 - 2*st**2 + st
                    h01 = -2*st**3 + 3*st**2;    h11 = st**3 - st**2
                    kprof = h00 * (-1.0) + h10 * m + h01 * 1.0 + h11 * m
                else:
                    kprof = -math.cos(math.pi * st)
                sww = math.sin(math.pi * st)
                dz = (self.lift + self.leg_lift[i]) * sww * sww
            # body-frame displacement d = (dx, dy):
            #   translation (0, amp*kprof) + yaw contribution om_amp*kprof * (z x r_foot),
            #   clamped like the firmware's OMEGA_AMP_CLAMP (3 cm)
            rx, ry = RADIAL[i]
            mx, my = MOUNTS[i]
            fx0, fy0 = mx + NEUTRAL_OUT * rx, my + NEUTRAL_OUT * ry
            # yaw as a TRUE rotation of the neutral foot about the body centre
            # (straight-tangent approximation reverses the turn at large strokes)
            th = om_amp * kprof + self.body_yaw
            cth, sth = math.cos(th), math.sin(th)
            ox = cth * fx0 - sth * fy0 - fx0
            oy = sth * fx0 + cth * fy0 - fy0
            dx_b = ampx * kprof + ox - self.body_dx
            dy_b = ampy * kprof + oy - self.body_dy - self.leg_dy[i]
            # -> leg frame: radial r=(rx,ry), tangential t = z x r = (-ry, rx)
            radial = dx_b * rx + dy_b * ry
            tang   = -dx_b * ry + dy_b * rx
            hx = NEUTRAL_OUT + radial
            coxa = math.atan2(tang, hx) + self.coxa_bias
            outdist = math.hypot(hx, tang)
            # body-attitude posing: feet follow the tilted body plane (the gimbal)
            tilt = ((fy0 - self.body_pivot_y) * math.tan(self.body_pitch)
                    - fx0 * math.tan(self.body_roll))
            # body_pivot_y: where the pitch rotates AROUND. 0 = body
            # center (walking servo, scans). The SUMMIT sets it to the
            # wall-anchored front feet (+0.19): shortening legs only
            # moves the body if they are LOADED — rotating about the
            # anchor makes mids AND rears fold in proportion, so the
            # whole body rotates up around the planted fronts
            _down_cmd = (NEUTRAL_DOWN + self.height
                         + self.leg_h[i] + tilt - dz)
            ik = leg_ik(outdist, _down_cmd)
            if os.environ.get("GARY_IK_PROBE") and i == 1:
                import sys as _sys
                self._pbk = getattr(self, "_pbk", 0) + 1
                if self._pbk % 400 == 0:
                    print("IKPROBE leg1 out=%.3f down=%.3f (nd=%.3f h=%.3f lh=%.3f tilt=%.3f dz=%.3f) ik=%s"
                          % (outdist, _down_cmd, NEUTRAL_DOWN,
                             self.height, self.leg_h[i], tilt, dz,
                             "OK" if ik is not None else "NONE"),
                          file=_sys.stderr)
            if ik is None and self.ram_full:
                # hip saturation: largest scale of the commanded body
                # shift THIS leg can execute (binary search the envelope)
                lo_s, hi_s, best = 0.0, 1.0, None
                for _ in range(7):
                    ms = (lo_s + hi_s) / 2.0
                    dxs, dys = dx_b * ms, dy_b * ms
                    rad2 = dxs * rx + dys * ry
                    tg2 = -dxs * ry + dys * rx
                    hx2 = NEUTRAL_OUT + rad2
                    ik2 = leg_ik(math.hypot(hx2, tg2), NEUTRAL_DOWN
                                 + self.height + self.leg_h[i] + tilt - dz)
                    if ik2 is not None:
                        best = (math.atan2(tg2, hx2) + self.coxa_bias, ik2)
                        lo_s = ms
                    else:
                        hi_s = ms
                if best is not None:
                    coxa, ik = best
            if ik is None:
                out[3*i:3*i+3] = self._last[3*i:3*i+3]   # hold, don't snap to neutral
                continue
            fem, knee = ik
            out[3 * i + 0] = coxa
            # model joints rotate about +y, which tilts links DOWN for positive q;
            # IK angles are up-positive -> negate both (FK-verified 2026-08-18).
            out[3 * i + 1] = -math.radians(fem - FEM0)
            out[3 * i + 2] = -math.radians(knee - KNEE0)
        self._last = out
        return out


def run(gait, stick, seconds, lift_cm=2.5, cadence_ms=None, speed_cap=None,
        touchdown="cosine", view=False, video=None, xml_out=None):
    xml = build_xml(lift_cm)
    if xml_out:
        open(xml_out, "w").write(xml)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    drv = GaitDriver(gait, stick, lift_cm, cadence_ms, speed_cap, touchdown)
    hold = GaitDriver(gait, 0.0, 0.0, cadence_ms, speed_cap).ctrl(0.0)  # lift 0 = true stand  # neutral stand pose
    settle = 1.0
    frames, renderer = [], None
    if video:
        renderer = mujoco.Renderer(model, 540, 960)
    viewer = None
    if view:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data)
    n = int((seconds + settle) / model.opt.timestep)
    wall0 = time.perf_counter()
    sat_steps, steps, tor_peak = 0, 0, 0.0
    y0 = None
    for s in range(n):
        t = s * model.opt.timestep
        data.ctrl[:] = drv.ctrl(model.opt.timestep) if t > settle else hold
        mujoco.mj_step(model, data)
        if t > settle:
            if y0 is None:
                y0 = data.body("base").xpos[1]
            steps += 1
            f = np.abs(data.actuator_force[[3 * i + 2 for i in range(6)]])
            tor_peak = max(tor_peak, float(f.max()))
            sat_steps += int((f > SERVO_TORQUE * 0.95).any())
        if viewer is not None:
            if not viewer.is_running():
                break
            viewer.sync()
            lag = (wall0 + t) - time.perf_counter()   # pace to real time
            if lag > 0:
                time.sleep(lag)
        if renderer is not None and s % 16 == 0:            # 30 fps-ish
            cam = mujoco.MjvCamera()
            cam.lookat[:] = data.body("base").xpos
            cam.distance, cam.elevation, cam.azimuth = 0.5, -18, 145 - 0.2 * s * model.opt.timestep * 60
            renderer.update_scene(data, camera=cam)
            frames.append(renderer.render().copy())
    if viewer is not None:
        viewer.close()
    dist = float(data.body("base").xpos[1] - y0) if y0 is not None else 0.0
    upright = float(data.body("base").xpos[2])
    res = dict(gait=gait, stick=stick, lift_cm=lift_cm, touchdown=touchdown,
               cadence_ms=cadence_ms or MIN_CADENCE_MS[GAITS.index(gait)],
               commanded_v_cm_s=round(drv.v * 100, 2),
               achieved_v_cm_s=round(dist / seconds * 100, 2),
               knee_torque_peak=round(tor_peak, 3),
               knee_sat_pct=round(100 * sat_steps / max(steps, 1), 1),
               body_height_end=round(upright, 3))
    if video and frames:
        import imageio
        imageio.mimwrite(video, frames, fps=30, quality=8)
        res["video"] = video
    return res


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--gait", default="WAVE", choices=GAITS)
    ap.add_argument("--stick", type=float, default=1.0)
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--lift", type=float, default=2.5)
    ap.add_argument("--cadence", type=int, default=None)
    ap.add_argument("--speed-cap", type=float, default=None)
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--video", default=None)
    ap.add_argument("--touchdown", default="cosine", choices=["cosine", "match"])
    a = ap.parse_args()
    r = run(a.gait, a.stick, a.seconds, a.lift, a.cadence, a.speed_cap, a.touchdown,
            view=a.view, video=a.video, xml_out=os.path.join(HERE, "gary.xml"))
    import json
    print(json.dumps(r, indent=1))
