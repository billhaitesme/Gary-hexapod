# GARY — Hiwonder miniHexa hexapod, custom firmware

> **About this public copy.** This is a cleaned snapshot of a private working repo. Hiwonder's
> factory firmware images, the WonderEcho voice-module firmware builds and audio clips, the
> CAD-derived simulation meshes (`sim/meshes/`), the LD19 CAD model, product photos and office
> documents are not included. Paths below that point at those files are kept for reference only.
> To run the MuJoCo twin, export the meshes from Hiwonder's published CAD with `cad_tools/`.

**Gary** is an 18-DOF Hiwonder miniHexa (ESP32 mainboard + MM32 servo controller) running
custom Arduino firmware instead of the factory image: a state-machine personality with voice
(WonderEcho), an LED-matrix face, R2-D2 buzzer sounds, PS5 DualSense control, three gaits,
a D500 LiDAR streaming 360° scans over WiFi to a PC map/dashboard, leg+gyro odometry, and a
WiFi-CSI "someone moved" detector that triggers a 3D nod-scan.

Bought 2026-03-30. Firmware written Apr 27–30 and Jul 8–17 2026. As of Aug 2026 the hardware
is in Norway with the rest of the droids and his knee servos are the open problem (see
[notes/STATUS-2026-08-18.md](notes/STATUS-2026-08-18.md) — **read that first**).

## Repo layout

```
firmware/
  GARY_v3/            <- THE live sketch (despite the number; v4 was a dead fork). Open GARY_v3.ino in Arduino IDE.
                         Copy wifi_secrets.example.h -> wifi_secrets.h and fill in your 2.4 GHz network.
  factory/            <- 4 MB factory backups (the safety net) + Apr-15 dumps
  history/            <- every earlier sketch: MINIHEXA_DIAG/REBUILD (Apr 9-11 brick recovery),
                         gary_v1 (v1.6, Apr 27-30), GARY_v2 (+PS5), GARY_v4 (dead Jul-10 fork),
                         GARY_v3_snapshot_2026-07-17 (the misnamed "gary_lidar_v2.cpp"), side quests
tools/                <- PC-side Python: LiDAR io/map/view/diag, odometry fit, web dashboard, launcher .bat
tools/robotpu/        <- July side quest: Gary as BLE remote for the ELECFREAKS PU micro:bit robot
voice/wonderecho/     <- WonderEcho (ChipIntelli CI1302) custom "Hey Gary" reflash attempt: vocab sheets,
                         built CI1302_GARY_v2_firmware.bin, generated sfw*.zip. Never landed on the module.
docs/                 <- HANDOVER_LOG (Apr 27), GARY_VOICE_SPEC, Quick Start + User Manual (Jul 11 — 2 days stale)
notes/                <- dated status / handoff notes (newest = current truth)
cad_tools/            <- Rhino .3dm parsers (uv run) for pulling ground-truth leg geometry — for the IK rebuild
action/               <- Hiwonder action-group .rob files (both are the all-1500 neutral placeholder)
photos/
```

Not versioned but living next to this README on disk: Hiwonder's full tutorial/software/CAD bundle
(`1. Tutorials`, `2. Software`, `3. Technical Parameter & Diagram` incl. the 494 MB `minihexa3 003.3dm`,
`FAQ`, `MiniHexa.exe`). Vendor GitHub clone: `C:\Users\BillH\Documents\MiniHexa`.
ChipIntelli SDK dumps: `C:\Users\BillH\Documents\Arduino\gary_v1\WonderEcjho Firmwares\`.

## Working copies

The Arduino IDE sketch Bill actually compiles from is `C:\Users\BillH\Documents\Arduino\GARY_v3\`.
This repo's `firmware/GARY_v3/` was copied from it on 2026-08-18 (with the WiFi password moved into
`wifi_secrets.h` in **both** places, and the build-breaking `gary_lidar_v2.cpp` renamed out of the
sketch in both places). Until the Documents folder is replaced by a junction to this repo, treat the
repo as canonical and re-copy after IDE edits.

## Quick facts

| | |
|---|---|
| Board | Hiwonder ESP32 mainboard, "ESP32 Dev Module", USB CH340 (COM14/COM15 historically) |
| Serial | 921600 baud (was 115200 before Jul 13) |
| Flash back to factory | `python -m esptool --port COMx write-flash 0 firmware/factory/newhexa_factory_backup.bin` |
| Servo controller | MM32, 18 PWM servos; GPIO18/19 shared with its PB3/PB4 — LiDAR RX on GPIO18 is a known, accepted risk |
| LiDAR | LDROBOT D500 (a.k.a. STL-19P), UART2 RX=GPIO18, 230400, TX-only sensor, motor PWM pin tied to GND |
| Network | STA only, joins home WiFi; TCP :3333 streams `SCAN:`/`NOD:`/`CSI:`/`RSSI:` lines; scanner mode is opt-in (`1`/`0`) |
| PC dashboard | `tools/Gary_Live_Map.bat` → `gary_web_dashboard.py --source tcp:<gary-ip>:3333 --port 8770` |
| Controller | PS5 DualSense via Bluepad32 (Start = PU relay toggle) |
| Voice | WonderEcho I²C 0x34, **factory** English firmware ("Hello Hiwonder"); custom "Hey Gary" reflash unfinished |
| Face | TM640B 16×8 matrix, CLK=33 DIN=32 |
| Sounds | LEDC ch0 pin 21 buzzer, R2-D2 frequency sweeps, `ENABLE_VOICE_PLAYBACK 0` |
| Never test on USB power | USB+battery dual power fakes low-battery, MM32 beeps, mic picks up ground loop. Flash → unplug → test on battery. |

## Sister projects

Kulya (hexapod, PCA9685) reuses lessons from here; the miniHexa **Standard** bought Apr 10 was parted
out as Gary's organ donor (~16 spare same-model servos + a spare mainboard, see
`NERD PARTS INVENTORY.md`).
