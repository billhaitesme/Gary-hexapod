# CAD tools for the miniHexa build model

Parse `..\3. Technical Parameter & Diagram\1. Diagram\minihexa3 003.3dm` (494MB, Rhino 8, mm, 947 objects on
colour layers, every part individually modeled, legs posed in a real standing pose).

Run: `uv run --with rhino3dm --with numpy python <script>` (uv.exe: `C:\Users\BillH\Documents\ComfyUI\.venv\Scripts\uv.exe`;
set `PYTHONIOENCODING=utf-8` — layer names are Chinese). Model read ≈15 s.

| Script | Purpose |
|---|---|
| `dump_parts.py` | pass 1 — one read of the model → `out/parts.json` (per object: layer, centroid, AABB, PCA extents+axes) |
| `leg_geometry.py` | pass 2 — servo shaft axes (hub + horn-disc normal), leg grouping, L1/L2 axis-to-axis, posed angles → `out/leg_geometry.json` |
| `foot_tip.py` | pass 3 — foot pad bottom → L3, tibia decomposition, body plate heights → `out/foot_tip.json` |
| `parse_minihexa_3dm.py`, `pca_minihexa_servos.py` | 2026-08-12 originals: layer survey + PCA servo dimensioning |

Results (2026-08-18): L1 29.20 / L2 37.83 / L3 74.35 mm; vendor `kinematics.h` was already within 0.7–2.4 mm.
See `../notes/CAD-GEOMETRY-2026-08-18.md` and `../firmware/GARY_v3/gary_geometry.h`.
