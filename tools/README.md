# PC-side tools (Python 3.10/3.14 were both used; needs numpy, pyserial, matplotlib)

All connect via `gary_lidar_io.open_source("COMx" | "tcp:<ip>:3333")` at 921600 baud and send `1` on
connect / `0` on exit to toggle Gary's scanner mode. Gary's LAN IP was 192.168.1.156 (DHCP, was .42).

| File | Status (2026-08-18) | Purpose |
|---|---|---|
| `Gary_Live_Map.bat` | works | double-click launcher → `gary_web_dashboard.py --source tcp:192.168.1.156:3333 --port 8770` |
| `gary_web_dashboard.py` | **current** | HTTP :8770 page: 3D cloud from `NOD:` + world-framed `SCAN:`, LiDAR "movers", RSSI/TRIP chart, scanner toggle. Places rings at Gary's REPORTED pose -> smears when the gyro heading drifts. |
| `gary_slam.py` | **new 2026-09-17** | scan-matching SLAM -- the "real fix" the odom fitter named. Takes HEADING from lidar scan-matching (drift-free) + leg odometry for translation, so the map holds fixed locations. `--selfcheck` proves it on a synthetic room (0.0mm vs 6.9m raw-odom); `--source tcp:gary.local:3333` runs live (top-down window: scan-matched path vs raw odom + the corrected cloud); `--source file:run.scan` replays a log. Reuses gary_odom_fit's ICP. |
| `gary_lidar_io.py` | works | shared serial/TCP shim |
| `gary_odom_fit.py` | current | v2 odometry calibrator — produced `ODOM_M` in the firmware; `--identity` gives the gyro yaw ratio (still to be applied) |
| `gary_pose_diag.py` | current | prints only x/y/theta/pitch/roll live |
| `gary_lidar_diag.py` | current | measures whether the 360-ring is a snapshot or a smear |
| `gary_live_dashboard.py` | works, superseded | matplotlib 3D cloud + RSSI strip |
| `nod_pointcloud.py` | works | offline `NOD:` log → 3D + top-down floor plan (also imported by web dashboard) |
| `gary_lidar_map.py` | **STALE** | expects 3-field pose; firmware emits 5 since Jul 13 → blank map |
| `gary_lidar_view.py` | **STALE** | expects no pose prefix at all → bearings shift |
| `gary_odom_calib.py` | superseded | v1 calibrator, "garbage" per odom_fit header |
| `gary_substitute*.ps1` | April, WonderEcho | swap Gary LM/TTS assets into the ChipIntelli SDK tree |
| `Create_Gary_Desktop_Shortcut.bat` | works | makes the Desktop "Gary Live Map" shortcut |
| `robotpu/` | July side quest | MakeCode/BLE sources for the ELECFREAKS PU micro:bit robot Gary remote-controls |
