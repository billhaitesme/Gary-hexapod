#!/usr/bin/env python3
"""
gary_odom_fit.py -- calibrate Gary's odometry by REGRESSION, not by two clean legs.

WHY THIS EXISTS (and why gary_odom_calib.py didn't work)
The old calibrator asked you to drive one perfectly straight 1-2m leg and one
perfect spin-in-place, then divided LiDAR-truth by odometry-claim. That method is
only as good as your ability to drive Gary perfectly, and his gait isn't tuned,
so you can't. Worse, it failed LOUDLY-but-USELESSLY: two big moves means ICP has
to match rings that overlap poorly, so the fit residual blew up to 200-250mm and
the numbers it printed were noise. Two runs gave ODOM_VY_SCALE = 0.62 and then
0.20, and an ODOM_OMEGA_SCALE that flipped sign. Garbage.

THE FIX: many small steps instead of two big ones.
Consecutive SCAN frames arrive ~5x/sec, so between them Gary has barely moved --
exactly the regime where ICP is rock solid (huge ring overlap, tiny rotation, no
local minima to fall into). We record hundreds of these little paired
measurements while you drive him around HOWEVER HE ACTUALLY MOVES, then
least-squares the calibration out of the whole pile.

Sloppy driving stops being a problem and becomes an asset: curving, drifting and
overshooting excite more of the input space, which makes the fit BETTER
conditioned, not worse. Drive him badly on purpose. Mix it up.

WHAT IT FITS: a 2x3 matrix, not two scalars.
A single ODOM_VY_SCALE assumes "commanded forward -> moves exactly forward." Your
own logs say otherwise: a commanded-straight leg came out rotated 9-14 deg. That
is gait cross-coupling, and NO value of a scalar scale can correct it. So we fit
the general linear map from commanded-velocity integrals to actual body-frame
displacement:

    [ dx_body ]   [ M00  M01 ] [ integral(vy)    ]
    [ dy_body ] = [ M10  M11 ] [ integral(omega) ]
    [ dtheta  ]   [ M20  M21 ]

  M00 = cm of forward travel per unit of commanded vy   (the old VY_SCALE)
  M21 = deg of turn per unit of commanded omega         (the old OMEGA_SCALE)
  M10 = SIDEWAYS drift per unit of commanded forward    (gait crabbing)
  M20 = YAW per unit of commanded forward               (gait veer) <-- your bug
  M01 = forward creep per unit of commanded turn        (spin isn't in-place)
  M11 = sideways creep per unit of commanded turn

The bottom-left terms are what "he won't walk straight" actually IS, expressed as
numbers. We can't make the legs behave, but we can stop the odometry from lying
about them.

GROUND TRUTH
  * yaw: circular cross-correlation of the two 360-point rings. A rotation shifts
    the whole range-vs-bearing profile by exactly the yaw.
  * translation: 2D POINT-TO-LINE ICP, seeded with that correlation yaw so it
    starts already de-rotated and can't wander into a local minimum. See the big
    comment on icp() for why point-to-point ICP silently under-reports motion --
    that mistake would have quietly biased every number this script prints.
  Every pair is scored and bad-residual pairs are DISCARDED rather than quietly
  poisoning the fit -- the old script's fatal flaw.

USAGE
    python gary_odom_fit.py --source tcp:192.168.1.156:3333

Then just drive. 60-120 seconds of varied motion. It prints a running count of
good samples. Press Ctrl-C (or wait for --target) and it prints the firmware
block to paste in.

DRIVE FOR VARIETY, NOT FOR PRECISION:
  * some forward and backward at several speeds
  * some spinning, both directions
  * some ARCS (forward + turn together) -- these are what pin down the
    off-diagonal terms; without them the fit is under-determined
  * pauses are fine and free (stationary pairs are filtered out)

CAVEATS, HONESTLY
  * Still dead reckoning. This makes the pose much less wrong; it does not make
    it right. Drift accumulates, there is no loop closure. The real fix is
    scan-matching the map itself, which is a bigger job.
  * Gait slip is speed- and surface-dependent and a linear model can't capture
    that. Calibrate at the speeds and on the surface you actually map on.
  * Odometry only integrates in CONTROLLER mode, so drive with the pad.
"""

import argparse
import math
import re
import sys
import time

import numpy as np

from gary_lidar_io import open_source, set_scanner_mode

SCAN_RE = re.compile(r"^SCAN:(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)(?:,(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?))?;(.+)$")

MIN_MM, MAX_MM = 120.0, 8000.0

# --- sample acceptance gates -------------------------------------------------
MIN_MOVE_MM = 4.0       # below this the pair is "standing still": no signal
MIN_TURN_DEG = 0.4
MAX_MOVE_MM = 400.0     # above this, ring overlap thins out -- don't trust ICP
MAX_TURN_DEG = 30.0
MAX_RESIDUAL_MM = 40.0  # point-to-line residual gate. Small steps SHOULD fit tight.
MIN_CORR_QUALITY = 0.35


def ring_points(dists):
    """360 ranges -> Nx2 (x,y) in robot body frame, mm. x forward, y left.

    The -sin is not a typo: the LiDAR indexes bearings clockwise, so negating y
    converts it into a normal right-handed frame -- the same one the firmware
    integrates odom_x/odom_y in."""
    d = np.asarray(dists, dtype=float)
    idx = np.arange(360)
    good = (d >= MIN_MM) & (d <= MAX_MM)
    d, ang = d[good], np.radians(idx[good])
    return np.stack([d * np.cos(ang), -d * np.sin(ang)], axis=1)


def yaw_by_correlation(d0, d1):
    """Yaw change (deg) from frame0 -> frame1, by circular cross-correlation."""
    a = np.asarray(d0, dtype=float).copy()
    b = np.asarray(d1, dtype=float).copy()
    for v in (a, b):
        bad = (v < MIN_MM) | (v > MAX_MM)
        v[bad] = np.nan
    if np.all(np.isnan(a)) or np.all(np.isnan(b)):
        return None, 0.0
    a = np.where(np.isnan(a), np.nanmean(a), a)
    b = np.where(np.isnan(b), np.nanmean(b), b)
    a -= a.mean()
    b -= b.mean()
    corr = np.real(np.fft.ifft(np.fft.fft(a) * np.conj(np.fft.fft(b))))
    shift = int(np.argmax(corr))
    if shift > 180:
        shift -= 360
    denom = np.linalg.norm(a) * np.linalg.norm(b)
    quality = float(corr.max() / denom) if denom else 0.0
    # The ring rotates OPPOSITE to the robot: if Gary yaws +10 deg, a feature
    # that used to read at bearing 100 now reads at bearing 90.
    return -float(shift), quality


def surface_normals(pts, max_gap_mm=250.0):
    """Unit surface normal at each ring point, plus a validity mask.

    The ring is stored in bearing order, so a point's array neighbours are also
    its neighbours in space -- the local tangent is just the vector between them.
    Where that vector is long we are straddling a depth discontinuity (a doorway,
    the edge of a table) and there is no meaningful surface there, so we mark it
    invalid rather than inventing a normal."""
    tang = np.roll(pts, -1, axis=0) - np.roll(pts, 1, axis=0)
    length = np.linalg.norm(tang, axis=1)
    ok = (length > 1e-6) & (length < max_gap_mm)
    safe = np.where(length > 1e-6, length, 1.0)
    tang = tang / safe[:, None]
    nrm = np.stack([-tang[:, 1], tang[:, 0]], axis=1)
    return nrm, ok


def icp(src, dst, init_yaw_deg=0.0, iters=25, tol=1e-3):
    """2D POINT-TO-LINE ICP mapping src -> dst. Returns (yaw_deg, t, rms).

    src = ring from the LATER frame, dst = ring from the EARLIER frame. The
    transform that lands one on the other IS Gary's motion, expressed in the
    earlier frame: p_earlier = R(dtheta) @ p_later + t.

    WHY POINT-TO-LINE AND NOT POINT-TO-POINT -- this one matters.
    Plain point-to-point ICP fails here, and it fails QUIETLY. The LiDAR samples
    every 1 degree, so along a wall 3m away the points sit ~50mm apart -- often
    further apart than the frame-to-frame motion we are trying to measure.
    Nearest-neighbour then just pairs each point with a DIFFERENT point on the
    same wall, and the fit happily slides the ring along that wall to zero out a
    residual that was never real. Measured against a synthetic room with known
    ground truth: a true 100mm step came back as 96mm, and a true
    (57mm, 4mm, 0.8deg) step came back as 12mm. It systematically UNDER-reports
    motion -- which would have quietly biased every scale this script prints,
    in the safe-looking direction where nothing obviously breaks.

    Point-to-line only penalises motion ALONG THE SURFACE NORMAL, so sliding a
    point along the wall it came from is free -- as, physically, it should be.
    The translation information then comes from corners and from walls at
    differing angles, which is also why a bare parallel-walled corridor is
    genuinely degenerate along its length. The residual gate catches those.

    Each iteration linearises the rotation (valid, because we are already seeded
    with the correlation yaw, so the leftover angle is small) and solves a
    3-parameter linear least squares for (dtheta, tx, ty)."""
    if len(src) < 30 or len(dst) < 30:
        return 0.0, np.zeros(2), float("inf")

    nrm, nrm_ok = surface_normals(dst)

    th = math.radians(init_yaw_deg)
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    t = np.zeros(2)
    cur = (R @ src.T).T

    prev = None
    rms = float("inf")
    for _ in range(iters):
        d2 = ((cur[:, None, :] - dst[None, :, :]) ** 2).sum(axis=2)
        j = d2.argmin(axis=1)
        nn2 = d2[np.arange(len(cur)), j]

        valid = nrm_ok[j]
        if valid.sum() < 30:
            break
        # trim the worst 25% of valid correspondences: movers, dropouts, and
        # geometry that only one of the two frames could actually see
        cut = np.quantile(nn2[valid], 0.75)
        keep = valid & (nn2 <= cut)
        if keep.sum() < 30:
            keep = valid

        P = cur[keep]            # source points, current estimate
        Q = dst[j[keep]]         # matched target points
        N = nrm[j[keep]]         # target surface normals

        # residual_i = n_i . (p_i + dtheta * perp(p_i) + t - q_i)  ->  0
        perp = np.stack([-P[:, 1], P[:, 0]], axis=1)
        A = np.stack([(N * perp).sum(axis=1), N[:, 0], N[:, 1]], axis=1)
        bvec = (N * (Q - P)).sum(axis=1)
        rms = float(np.sqrt((bvec ** 2).mean()))   # point-to-LINE residual

        try:
            sol, *_ = np.linalg.lstsq(A, bvec, rcond=None)
        except np.linalg.LinAlgError:
            break
        dth, tx, ty = float(sol[0]), float(sol[1]), float(sol[2])
        if not np.isfinite([dth, tx, ty]).all() or abs(dth) > 0.5:
            break   # wild step: degenerate, corridor-like scene

        c, s = math.cos(dth), math.sin(dth)
        Ri = np.array([[c, -s], [s, c]])
        ti = np.array([tx, ty])
        cur = (Ri @ cur.T).T + ti
        R = Ri @ R
        t = Ri @ t + ti

        if prev is not None and abs(prev - rms) < tol:
            break
        prev = rms

    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    return yaw, t, rms


def to_midpoint_frame(t, yaw_deg):
    """Re-express ICP's translation in the MIDPOINT heading frame.

    ICP hands back the displacement measured in the EARLIER frame. The firmware
    integrates with a midpoint rule -- it rotates half, translates, rotates half --
    so the body displacement it wants is the one expressed halfway through the
    turn. Rotating by -yaw/2 converts between them.

    Small (yaw is a degree or two per sample) but not zero, and it is a
    SYSTEMATIC error correlated with turning, so it biases the coupled terms
    rather than just adding noise. Fixing it lifted the sideways-drift row's R^2
    from 0.79 to 0.87 in the synthetic test. Cheap, so do it right."""
    h = math.radians(-yaw_deg / 2.0)
    c, s = math.cos(h), math.sin(h)
    return float(c * t[0] - s * t[1]), float(s * t[0] + c * t[1])


def wrap180(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def odom_delta_body(p0, p1, vy_scale, omega_scale):
    """From two odometry poses, recover the COMMANDED velocity integrals.

    The firmware integrates: theta += omega*OMEGA_SCALE*dt, then moves
    vy*VY_SCALE*dt along the midpoint heading. Inverting that:

        b = integral(omega_units * dt) = dtheta_odom / OMEGA_SCALE
        a = integral(vy_units    * dt) = distance_travelled / VY_SCALE

    We take the SIGNED distance along the midpoint heading: odometry can only
    ever move along its own heading, so the perpendicular component is zero by
    construction and carries no information.

    Returns (a, b, fwd_mm, dtheta_deg) -- the last two are the raw odometry
    claim, used for the acceptance gates."""
    dth = wrap180(p1[2] - p0[2])
    dx_w = (p1[0] - p0[0]) * 10.0   # cm -> mm
    dy_w = (p1[1] - p0[1]) * 10.0
    th_mid = math.radians(p0[2] + dth / 2.0)
    fwd_mm = dx_w * math.cos(th_mid) + dy_w * math.sin(th_mid)

    a = (fwd_mm / 10.0) / vy_scale if vy_scale else 0.0   # cm-units of the old scale
    b = dth / omega_scale if omega_scale else 0.0
    return a, b, fwd_mm, dth


def odom_body_delta(p0, p1):
    """Full body-frame delta between two odometry poses, in the MIDPOINT frame:
    (dx_mm, dy_mm, dtheta_deg).

    Unlike odom_delta_body() above, this does NOT assume the old scalar model --
    it just measures what odometry says happened, including any lateral component.
    Once the ODOM_M matrix is flashed, odometry CAN move sideways, so the old
    "project onto heading, discard the rest" inversion would silently throw away
    exactly the term we just taught it about."""
    dth = wrap180(p1[2] - p0[2])
    dx_w = (p1[0] - p0[0]) * 10.0
    dy_w = (p1[1] - p0[1]) * 10.0
    th_mid = math.radians(p0[2] + dth / 2.0)
    c, s = math.cos(th_mid), math.sin(th_mid)
    return dx_w * c + dy_w * s, -dx_w * s + dy_w * c, dth


class Reader:
    """Non-blocking line reader.

    Unlike the old calibrator this NEVER lets a backlog build: we consume the
    stream continuously, so every frame we look at is one Gary just produced. The
    old script read a line only when you pressed ENTER, by which time the socket
    held dozens of seconds-old frames -- and it handed back the OLDEST."""

    def __init__(self, src):
        self.src = src
        self.buf = b""

    def frames(self):
        while getattr(self.src, "in_waiting", False):
            chunk = self.src.readline()
            if not chunk:
                break
            self.buf += chunk
            while b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                m = SCAN_RE.match(line.decode("utf-8", errors="replace").strip())
                if not m:
                    continue
                try:
                    dists = [int(x) for x in m.group(6).split(",")]
                except ValueError:
                    continue
                if len(dists) != 360:
                    continue
                pose = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
                yield pose, dists


def fit(rows_in, rows_out):
    """Least squares rows_out ~= rows_in @ M.T, with NO intercept -- zero command
    must give zero motion, that's physics, not something to fit. Returns M (3x2)
    and per-output R^2."""
    A = np.asarray(rows_in)     # N x 2   [a, b]
    Y = np.asarray(rows_out)    # N x 3   [dx_mm, dy_mm, dtheta_deg]
    M, *_ = np.linalg.lstsq(A, Y, rcond=None)   # 2 x 3
    M = M.T                                     # 3 x 2
    pred = A @ M.T
    r2 = []
    for k in range(3):
        ss_res = float(((Y[:, k] - pred[:, k]) ** 2).sum())
        ss_tot = float(((Y[:, k] - Y[:, k].mean()) ** 2).sum())
        r2.append(1.0 - ss_res / ss_tot if ss_tot > 1e-9 else float("nan"))
    return M, r2


FIRMWARE_TEMPLATE = """
// LiDAR-regressed LEG odometry. The inputs are no longer the joystick: they are
// the two legs' stroke rates (amplitude / step-period), which is what a walking
// robot's speed actually IS. send_vy could never work here -- the leg amplitude
// saturates at 3.0cm while the step period independently slides 1200->400ms, so
// speed is grossly non-linear in the stick. Rows are body-frame dx(cm), dy(cm),
// dtheta(deg); columns are the integrated leg2 and leg5 stroke rates.
static float ODOM_M[3][2] = {
  { %+10.5ff, %+10.5ff },   // dx_cm      per (leg2_rate, leg5_rate)
  { %+10.5ff, %+10.5ff },   // dy_cm      per (leg2_rate, leg5_rate)
  { %+10.5ff, %+10.5ff },   // dtheta_deg per (leg2_rate, leg5_rate)
};

void odom_update(float vy_units, float omega_units) {
  uint32_t now = millis();
  if (odom_last_update_ms == 0) { odom_last_update_ms = now; return; }
  float dt = (now - odom_last_update_ms) / 1000.0f;
  odom_last_update_ms = now;
  if (dt <= 0.0f || dt > 1.0f) return;   // 0.5s was too tight at a 4Hz loop

  float a = vy_units    * dt;   // now = leg2 amplitude/period, integrated
  float b = omega_units * dt;   // now = leg5 amplitude/period, integrated

  float dx_body = ODOM_M[0][0] * a + ODOM_M[0][1] * b;   // cm, forward
  float dy_body = ODOM_M[1][0] * a + ODOM_M[1][1] * b;   // cm, left
  float dth     = ODOM_M[2][0] * a + ODOM_M[2][1] * b;   // deg

  // Midpoint integration: matters more now that translation and rotation are
  // coupled -- a step that both moves and turns lands between the two headings.
  float th0 = odom_theta_deg * (PI / 180.0f);
  odom_theta_deg += dth;
  while (odom_theta_deg > 180.0f)  odom_theta_deg -= 360.0f;
  while (odom_theta_deg < -180.0f) odom_theta_deg += 360.0f;
  float th_mid = (th0 + odom_theta_deg * (PI / 180.0f)) / 2.0f;

  float c = cosf(th_mid), s = sinf(th_mid);
  odom_x += dx_body * c - dy_body * s;
  odom_y += dx_body * s + dy_body * c;
}
"""


def report(M, r2, n):
    # M maps [a, b] -> [dx_mm, dy_mm, dtheta_deg]. The firmware wants cm.
    Mcm = M.copy()
    Mcm[0, :] /= 10.0
    Mcm[1, :] /= 10.0

    print("\n" + "=" * 72)
    print("FIT from %d good sample pairs" % n)
    print("=" * 72)
    print("  INPUTS are now the two leg stroke-rates (amplitude/step-period), NOT the")
    print("  stick. Speed is stride/period; send_vy is neither, and saturates at 3.0cm.")
    print()
    print("                            per leg2 rate   per leg5 rate")
    print("  forward  dx (cm)     %+12.4f   %+12.4f" % (Mcm[0, 0], Mcm[0, 1]))
    print("  sideways dy (cm)     %+12.4f   %+12.4f" % (Mcm[1, 0], Mcm[1, 1]))
    print("  yaw   dtheta (deg)   %+12.4f   %+12.4f" % (Mcm[2, 0], Mcm[2, 1]))
    print()
    print("  fit quality (R^2):  dx %.2f   dy %.2f   dtheta %.2f" % tuple(r2))

    # A NULL fit -- every coefficient ~0 AND every R^2 ~0 -- is a completely
    # different disease from a merely noisy one, and must not be reported as
    # "drive more". It means the LiDAR insists Gary did not move, whatever the
    # odometry claimed, i.e. the two rings being compared were near-identical.
    # (This is exactly what happened on the first real run, and it turned out the
    # ring was a ~2.6s smear rather than a snapshot -- see gary_lidar_diag.py.)
    tiny = float(np.abs(Mcm).max()) < 0.05
    flat = max(r2) < 0.10
    if tiny and flat:
        print()
        print("  !! NULL FIT -- every coefficient is ~0 and nothing is explained.")
        print("     This is NOT 'drive more'. It means the LiDAR says Gary did not move,")
        print("     no matter what odometry claimed -- i.e. consecutive rings were")
        print("     essentially IDENTICAL. Almost certainly the ring is not a snapshot:")
        print("     it is a slow mosaic, stale enough that consecutive frames share most")
        print("     of the same numbers and ICP correctly reports zero motion.")
        print()
        print("     Run this before doing anything else:")
        print("         python gary_lidar_diag.py --source <same source>")
    elif min(r2) < 0.5:
        print("  ! At least one row fits poorly. Usually means not enough VARIETY in the")
        print("    driving -- especially ARCS (forward+turn together), which are the only")
        print("    motions that separate the coupled terms. Drive more, re-run.")

    # Rotate into the coordinates that actually mean something.
    #
    # The two inputs are the left and right leg stroke-rates. Change basis to
    # their SUM and DIFFERENCE and you get the two things a walker really has:
    # a "drive" mode and a "turn" mode. Which is which is set by Hiwonder's
    # mirrored left/right leg frames -- so DETECT it, don't assume it. (I assumed
    # it once and got it exactly backwards.)
    #
    #   u_a = s + d ,  u_b = s - d      s = sum/2 , d = difference/2
    #   any row  [p, q]  becomes  (p+q)*s + (p-q)*d
    S = Mcm[:, 0] + Mcm[:, 1]     # response to the SYMMETRIC mode
    D = Mcm[:, 0] - Mcm[:, 1]     # response to the ANTISYMMETRIC mode

    # Whichever mode drives forward motion is the "drive" mode; the other turns.
    if abs(D[0]) >= abs(S[0]):
        drive, turn = D, S
        mode_txt = "DIFFERENCE of the leg strokes drives him; their SUM turns him"
    else:
        drive, turn = S, D
        mode_txt = "SUM of the leg strokes drives him; their DIFFERENCE turns him"

    print()
    print("  DECOUPLED (%s):" % mode_txt)
    print("    drive mode  ->  %+7.3f cm fwd   %+7.3f cm side   %+7.3f deg yaw"
          % (drive[0], drive[1], drive[2]))
    print("    turn  mode  ->  %+7.3f cm fwd   %+7.3f cm side   %+7.3f deg yaw"
          % (turn[0], turn[1], turn[2]))

    if abs(drive[0]) > 1e-6 and abs(turn[2]) > 1e-6:
        veer = drive[2] / drive[0]              # deg of yaw per cm walked
        orbit = turn[1] / turn[2]               # cm sideways per deg turned
        creep = turn[0] / turn[2]               # cm forward per deg turned
        print()
        print("  WHAT THAT MEANS IN PLAIN ENGLISH -- this is Gary's limp, in numbers:")
        print("    VEER : walking straight, he yaws %+.1f deg per METRE travelled."
              % (veer * 100.0))
        print("    ORBIT: spinning 'in place', he slides %+.1f cm sideways per 90 deg."
              % (orbit * 90.0))
        print("    CREEP: spinning 'in place', he also drifts %+.1f cm forward per 90 deg."
              % (creep * 90.0))
        print()
        print("  Sanity check this against reality before trusting it. Your very first")
        print("  calibration attempt failed because he 'rotated 9-14 deg' during what was")
        print("  supposed to be a straight 1-2m leg. This fit -- from completely different")
        print("  data, by a completely different method -- independently says %.0f deg/m."
              % abs(veer * 100.0))
        print()
        print("  The matrix does NOT fix any of this. It stops the odometry being")
        print("  surprised by it, which is what makes the map work. He still walks")
        print("  crooked. Straightening him is gait tuning: a separate job, but now")
        print("  you have a number to tune AGAINST instead of eyeballing it.")

    print("\n" + "=" * 72)
    print("PASTE INTO GARY_v3.ino -- replaces ODOM_VY_SCALE/ODOM_OMEGA_SCALE and")
    print("odom_update() (around line 2002)")
    print("=" * 72)
    print(FIRMWARE_TEMPLATE % (Mcm[0, 0], Mcm[0, 1], Mcm[1, 0], Mcm[1, 1], Mcm[2, 0], Mcm[2, 1]))
    print("=" * 72)
    print("\nAfter reflashing, re-run this with --identity. A correct fit comes back")
    print("as the IDENTITY model: M00 ~ 1.0, M21 ~ 1.0, cross terms ~ 0 -- because")
    print("odometry is now already telling the truth and LiDAR has nothing left to")
    print("correct. If it doesn't, the leftover error isn't linear.\n")


def report_verify(odom, lidar, n):
    """VERIFY mode: odometry should now already agree with the LiDAR, so instead
    of fitting a correction we just measure how big the disagreement still is."""
    O = np.asarray(odom)      # N x 3   what odometry claimed
    L = np.asarray(lidar)     # N x 3   what the LiDAR saw
    E = L - O

    print("\n" + "=" * 72)
    print("VERIFY -- %d sample pairs. Odometry vs LiDAR, per step." % n)
    print("=" * 72)
    names = ("forward (mm)", "sideways (mm)", "yaw (deg)")
    for k in range(3):
        bias = float(E[:, k].mean())
        rms = float(np.sqrt((E[:, k] ** 2).mean()))
        denom = float(np.abs(O[:, k]).mean())
        pct = 100.0 * abs(bias) / denom if denom > 1e-6 else float("nan")
        print("  %-14s  bias %+8.2f   rms %7.2f   (bias is %.0f%% of typical step)"
              % (names[k], bias, rms, pct))

    # The headline: does odometry still systematically under/over-shoot?
    # Least-squares slope through the origin, NOT the mean of per-sample ratios --
    # a mean of ratios is dominated by the samples where the denominator is nearly
    # zero, which is exactly where the ratio is meaningless.
    def slope(o, l):
        den = float((o * o).sum())
        return float((o * l).sum() / den) if den > 1e-9 else float("nan")
    ratio_f = slope(O[:, 0], L[:, 0])
    ratio_t = slope(O[:, 2], L[:, 2])
    print()
    print("  LiDAR/odometry ratio   forward %.3f    yaw %.3f    (want ~1.00 both)"
          % (ratio_f, ratio_t))
    print()
    if abs(ratio_f - 1.0) < 0.12 and abs(ratio_t - 1.0) < 0.12:
        print("  GOOD -- the matrix is doing its job. Remaining error is per-step noise,")
        print("  which still accumulates as drift, but it is no longer a systematic lie.")
    else:
        print("  STILL BIASED. The leftover error is not a constant linear scale --")
        print("  most likely gait slip that varies with speed. Re-fit while driving at")
        print("  the speed you actually map at, or accept a nonlinear model.")
    print("=" * 72 + "\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="tcp:192.168.1.156:3333")
    ap.add_argument("--stride", type=int, default=3,
                    help="frames between the two rings of a pair (default 3, ~0.6s). "
                         "Bigger = more motion per sample = better signal, but thinner "
                         "ring overlap for ICP.")
    ap.add_argument("--target", type=int, default=250,
                    help="stop automatically after this many GOOD samples")
    ap.add_argument("--vy-scale", type=float, default=1.0,
                    help="ODOM_VY_SCALE currently flashed (default 1.0)")
    ap.add_argument("--omega-scale", type=float, default=1.0,
                    help="ODOM_OMEGA_SCALE currently flashed (default 1.0)")
    ap.add_argument("--identity", action="store_true",
                    help="firmware already runs the ODOM_M matrix (i.e. you're "
                         "verifying a previous fit, not doing the first one)")
    ap.add_argument("--save", default=None, help="also dump raw samples to this .csv")
    args = ap.parse_args()

    print("Connecting to %s ..." % args.source)
    src = open_source(args.source)
    set_scanner_mode(src, True)
    time.sleep(0.5)
    print("Connected, scanner mode ON.\n")

    print("=" * 72)
    print("DRIVE GARY NOW -- with the PS5 pad (odometry only runs in CONTROLLER mode).")
    print()
    print("  Do NOT try to drive well. Drive VARIED:")
    print("    * forward and backward at a few different speeds")
    print("    * spin both directions")
    print("    * ARCS -- forward and turning at the same time. These matter most:")
    print("      without them the coupled terms can't be separated at all.")
    print("    * pauses are free; standing-still pairs are discarded automatically")
    print()
    print("  Target: %d good samples. Ctrl-C at any time to fit what we have." % args.target)
    print("=" * 72 + "\n")

    reader = Reader(src)
    hist = []
    rows_in, rows_out = [], []
    rejected = {"still": 0, "toobig": 0, "residual": 0, "corr": 0}
    last_print = 0.0

    try:
        while len(rows_in) < args.target:
            got = False
            for pose, dists in reader.frames():
                got = True
                hist.append((pose, dists))
                if len(hist) <= args.stride:
                    continue
                p0, d0 = hist[-1 - args.stride]
                p1, d1 = hist[-1]
                del hist[:-(args.stride + 1)]

                if args.identity:
                    fwd_mm, lat_mm, dth_odom = odom_body_delta(p0, p1)
                    a, b = fwd_mm, dth_odom      # unused in verify mode
                else:
                    a, b, fwd_mm, dth_odom = odom_delta_body(p0, p1, args.vy_scale,
                                                             args.omega_scale)
                    lat_mm = 0.0
                if abs(fwd_mm) < MIN_MOVE_MM and abs(dth_odom) < MIN_TURN_DEG:
                    rejected["still"] += 1
                    continue
                if abs(fwd_mm) > MAX_MOVE_MM or abs(dth_odom) > MAX_TURN_DEG:
                    rejected["toobig"] += 1
                    continue

                yaw_c, q = yaw_by_correlation(d0, d1)
                if yaw_c is None or q < MIN_CORR_QUALITY:
                    rejected["corr"] += 1
                    continue

                A = ring_points(d0)   # earlier
                B = ring_points(d1)   # later
                yaw_true, t_true, rms = icp(B, A, init_yaw_deg=yaw_c)
                if not np.isfinite(rms) or rms > MAX_RESIDUAL_MM:
                    rejected["residual"] += 1
                    continue

                dx_mid, dy_mid = to_midpoint_frame(t_true, yaw_true)
                if args.identity:
                    rows_in.append([fwd_mm, lat_mm, dth_odom])   # odometry's claim
                else:
                    rows_in.append([a, b])
                rows_out.append([dx_mid, dy_mid, float(yaw_true)])

            now = time.time()
            if now - last_print > 1.0:
                last_print = now
                print("\r  good %4d / %d   |  rejected: still %d  too-big %d  bad-fit %d  weak-corr %d   "
                      % (len(rows_in), args.target, rejected["still"], rejected["toobig"],
                         rejected["residual"], rejected["corr"]), end="", flush=True)
            if not got:
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\n  Stopped by you.")

    print()
    set_scanner_mode(src, False)
    src.close()

    if len(rows_in) < 20:
        print("\n  Only %d usable samples -- not enough to fit anything." % len(rows_in))
        print("  Almost certainly one of:")
        print("    * you didn't actually drive him (odom only integrates in CONTROLLER mode)")
        print("    * scanner mode isn't on / no SCAN: frames arriving")
        print("    * the space is too featureless for ICP to lock on")
        sys.exit(1)

    if args.save:
        arr = np.hstack([np.asarray(rows_in), np.asarray(rows_out)])
        np.savetxt(args.save, arr, delimiter=",",
                   header="a_vy_integral,b_omega_integral,dx_mm,dy_mm,dtheta_deg",
                   comments="")
        print("  raw samples -> %s" % args.save)

    if args.identity:
        report_verify(rows_in, rows_out, len(rows_in))
    else:
        M, r2 = fit(rows_in, rows_out)
        report(M, r2, len(rows_in))


if __name__ == "__main__":
    main()
