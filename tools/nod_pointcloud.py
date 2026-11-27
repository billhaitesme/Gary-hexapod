#!/usr/bin/env python3
"""
nod_pointcloud.py

Parses Gary's nod-scan serial output and reconstructs a 3D point cloud.

Coordinate frame (robot body frame, millimeters):
    x = forward, y = left, z = up
Gary sits at the origin (0, 0, 0), marked in every output. The "forward"
arrow in the top-down floor plan shows the direction he was facing at the
moment the scan started (pitch=roll=0).

Orientation note: judging up/down or mirroring by freely rotating an
interactive 3D scatter is unreliable -- it's very easy to drag past a pole
and end up looking from underneath without realizing it, especially with
an axis box that isn't equal-aspect (a compressed Z range gets stretched
to fill the view, which compounds the confusion). To settle orientation
unambiguously, this script now also renders a flat top-down floor plan
(orthographic projection onto the X-Y plane, no camera/rotation involved)
alongside the two 3D views. If the floor plan's shape and Gary's facing
arrow don't match the real room when compared like a blueprint, the fix
is a sign flip on the specific axis that's wrong -- tell me which one.
"""

import argparse
import re
import sys
from pathlib import Path

import numpy as np

NOD_RE = re.compile(r"^NOD:(-?\d+(?:\.\d+)?)(?:,(-?\d+(?:\.\d+)?))?;(.+)$")

MAX_RANGE_MM = 4000
MIN_RANGE_MM = 30


def parse_log(path):
    scans = []
    with open(path, "r", errors="replace") as f:
        for line in f:
            line = line.strip()
            m = NOD_RE.match(line)
            if not m:
                continue
            pitch = float(m.group(1))
            roll = float(m.group(2)) if m.group(2) is not None else 0.0
            parts = m.group(3).split(",")
            try:
                dists = [int(p) for p in parts]
            except ValueError:
                continue
            if len(dists) != 360:
                dists = (dists + [0] * 360)[:360]
            scans.append((pitch, roll, dists))
    return scans


def scans_to_pointcloud(scans):
    # Axis corrections, pinned down against real hardware via the labeled
    # compass gizmo in the interactive view (ground truth: compare the
    # colored spokes to Gary's actual left/right and up/down):
    #   - Y (left) was mirrored -- what the code called "left" was Gary's
    #     right. Final Y is negated to correct this.
    #   - Z (up) needed NO change after all -- an earlier fix negated Z
    #     based on a misread of a freely-rotated, non-equal-aspect 3D view
    #     (easy to lose track of which way is up while dragging). That
    #     negation has been reverted; z2 is used as originally derived.
    pts = []
    for pitch_deg, roll_deg, dists in scans:
        theta = np.radians(pitch_deg)
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        phi = np.radians(roll_deg)
        cos_r, sin_r = np.cos(phi), np.sin(phi)
        for i, d in enumerate(dists):
            if d < MIN_RANGE_MM or d > MAX_RANGE_MM:
                continue
            ang = np.radians(i)
            x = d * np.cos(ang)
            y = d * np.sin(ang)
            y1 = y * cos_r
            z1 = y * sin_r
            x2 = x * cos_t + z1 * sin_t
            z2 = -x * sin_t + z1 * cos_t
            y2 = y1
            pts.append((x2, -y2, z2))
    return np.array(pts, dtype=float)


def _equal_box_aspect(ax, pts):
    # Force true equal-unit proportions on a 3D matplotlib axis -- without
    # this, matplotlib stretches whichever axis has the smallest data range
    # to fill the same box size as the others, which badly distorts shape
    # and makes orientation easy to misjudge.
    spans = pts.max(axis=0) - pts.min(axis=0)
    spans = np.where(spans < 1, 1, spans)
    ax.set_box_aspect(tuple(spans))


def save_static_png(pts, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=1.5, c=pts[:, 2], cmap="viridis")
    ax.scatter([0], [0], [0], s=200, c="red", marker="^", edgecolors="black", linewidths=1, depthshade=False)
    ax.text(0, 0, 0, "GARY", color="red", fontsize=11, fontweight="bold")
    ax.set_xlabel("X forward (mm)")
    ax.set_ylabel("Y left (mm)")
    ax.set_zlabel("Z up (mm)")
    ax.set_title(f"Gary nod-scan point cloud ({len(pts)} pts)")
    ax.view_init(elev=25, azim=-60)
    _equal_box_aspect(ax, pts)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def save_topdown_png(pts, out_path):
    # Flat orthographic floor plan, looking straight down the Z axis.
    # No camera rotation, no perspective -- read it like a blueprint.
    # Standard screen convention: X forward -> right, Y left -> up.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 8))
    sc = ax.scatter(pts[:, 0], pts[:, 1], s=2, c=pts[:, 2], cmap="viridis", alpha=0.8)
    ax.scatter([0], [0], s=260, c="red", marker="D", edgecolors="black", linewidths=1.2, zorder=5)
    ax.annotate("GARY", (0, 0), textcoords="offset points", xytext=(12, 10),
                color="red", fontsize=13, fontweight="bold", zorder=6)
    # Forward-facing arrow: direction Gary was pointed at scan start (pitch=roll=0).
    arrow_len = max(200.0, 0.08 * (pts[:, 0].max() - pts[:, 0].min() if len(pts) else 1000))
    ax.annotate("", xy=(arrow_len, 0), xytext=(0, 0),
                arrowprops=dict(arrowstyle="-|>", color="red", lw=2), zorder=6)
    ax.text(arrow_len * 1.1, 0, "forward\n(heading at scan start)", color="red", fontsize=8, va="center")
    ax.set_xlabel("X forward (mm)")
    ax.set_ylabel("Y left (mm)")
    ax.set_title("Top-down floor plan (looking straight down, no rotation)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    fig.colorbar(sc, ax=ax, label="Z up (mm)", shrink=0.8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def save_interactive_html(pts, out_path):
    import plotly.graph_objects as go

    # Compass gizmo at Gary's position: fixed-length axis spokes so
    # orientation stays readable no matter how the camera gets dragged.
    # Red = forward (+X), green = left (+Y), blue = up (+Z).
    span = float(np.max(pts.max(axis=0) - pts.min(axis=0))) if len(pts) else 1000.0
    spoke = max(150.0, 0.15 * span)

    def axis_spoke(direction, color, label):
        end = np.array(direction) * spoke
        return go.Scatter3d(
            x=[0, end[0]], y=[0, end[1]], z=[0, end[2]],
            mode="lines+text",
            line=dict(color=color, width=8),
            text=["", label],
            textposition="top center",
            textfont=dict(color=color, size=13),
            name=label,
            showlegend=False,
        )

    fig = go.Figure(
        data=[
            go.Scatter3d(
                x=pts[:, 0], y=pts[:, 1], z=pts[:, 2],
                mode="markers",
                marker=dict(size=1.5, color=pts[:, 2], colorscale="Viridis", opacity=0.8),
                name="scan",
                showlegend=False,
            ),
            go.Scatter3d(
                x=[0], y=[0], z=[0],
                mode="markers+text",
                marker=dict(size=8, color="red", symbol="diamond", line=dict(color="black", width=1)),
                text=["GARY"],
                textposition="bottom center",
                textfont=dict(color="red", size=14),
                name="Gary",
                showlegend=False,
            ),
            axis_spoke((1, 0, 0), "red", "FWD"),
            axis_spoke((0, 1, 0), "green", "LEFT"),
            axis_spoke((0, 0, 1), "blue", "UP"),
        ]
    )
    fig.update_layout(
        scene=dict(
            xaxis_title="X forward (mm)",
            yaxis_title="Y left (mm)",
            zaxis_title="Z up (mm)",
            aspectmode="data",
            camera=dict(eye=dict(x=1.4, y=1.4, z=1.1), up=dict(x=0, y=0, z=1)),
            # "turntable" keeps the up-axis locked while dragging, which is
            # what makes left-drag=rotate / right-drag=pan / wheel=zoom behave
            # predictably. Left as the explicit default (rather than relying
            # on plotly's implicit default) since browsers vary in how they
            # otherwise route wheel/right-click events on a 3D scene.
            dragmode="turntable",
        ),
        title=(
            f"Gary nod-scan point cloud ({len(pts)} pts) -- "
            "left-drag: rotate | scroll wheel: zoom | right-drag (or shift+drag): pan | "
            "red/green/blue spokes at Gary = forward/left/up"
        ),
        margin=dict(l=0, r=0, b=0, t=40),
        showlegend=False,
    )
    # scrollZoom=True makes the mouse wheel zoom the scene instead of being
    # swallowed by page scroll -- without it, some browsers only zoom on a
    # slow right-click-drag, which is exactly the "wheel doesn't zoom, right
    # mouse button barely works" behavior this fixes.
    fig.write_html(
        out_path,
        include_plotlyjs="cdn",
        config={"scrollZoom": True, "displaylogo": False},
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", required=True)
    ap.add_argument("--out", default="nod_cloud")
    args = ap.parse_args()

    scans = parse_log(args.log)
    if not scans:
        print(f"No NOD: lines found in {args.log}", file=sys.stderr)
        sys.exit(1)

    print(f"Parsed {len(scans)} slices: {[(p, r) for p, r, _ in scans]}")
    pts = scans_to_pointcloud(scans)
    dropped = len(scans) * 360 - len(pts)
    print(f"Reconstructed {len(pts)} 3D points (dropped {dropped} zero/out-of-range returns)")

    png_path = str(Path(args.out).with_suffix(".png"))
    topdown_path = str(Path(args.out).parent / (Path(args.out).stem + "_topdown.png"))
    html_path = str(Path(args.out).with_suffix(".html"))
    save_static_png(pts, png_path)
    save_topdown_png(pts, topdown_path)
    save_interactive_html(pts, html_path)
    print(f"Wrote {png_path}")
    print(f"Wrote {topdown_path} (unambiguous top-down floor plan)")
    print(f"Wrote {html_path} (open in a browser, drag to rotate)")


if __name__ == "__main__":
    main()
