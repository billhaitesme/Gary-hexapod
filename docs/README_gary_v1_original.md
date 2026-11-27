# Gary v1 — Firmware for the MiniHexa K12 v1.0

This is custom Arduino firmware for the Hiwonder MiniHexa hexapod, replacing
the factory firmware with a state-machine-driven robot personality named Gary.

## What's in this folder

```
gary_v1/
├── gary_v1.ino       Main sketch (state machine, behaviors)
├── gary_config.h     All tunable values (timings, thresholds, eye colors, voice IDs)
└── README.md         This file
```

## Before you compile

This sketch depends on Hiwonder's MiniHexa libraries. You need them in the
same folder as `gary_v1.ino`. Specifically, copy these files from any of the
example projects you cloned (e.g.,
`MiniHexa/arduino-programming-projects/4.2 运动控制基本教程/5.2 全向运动/omnidirectional_movement/`)
into the same folder as `gary_v1.ino`:

- `hiwonder_robot.h` / `hiwonder_robot.cpp`
- `hiwonder_board.h` / `hiwonder_board.cpp`
- `hiwonder_servo.h` / `hiwonder_servo.cpp`
- `hiwonder_sensor.h` / `hiwonder_sensor.cpp`
- `hiwonder_i2c.h` / `hiwonder_i2c.cpp`
- `MadgwickAHRS.h` / `MadgwickAHRS.cpp`
- `WMMatrixLed.h` / `WMMatrixLed.cpp`
- `global.h`

The `kinematics.h` referenced inside `hiwonder_robot.h` should be installed
as part of Hiwonder's Arduino library bundle. Follow Hiwonder's
"4.1 Arduino IDE Installation & Servo Calibration" tutorial if you haven't
yet — it walks through setting up the ESP32 board package and the Hiwonder
library install.

## Flashing

1. **Make sure you have your factory backup safe.**
   Should be at `C:\Users\BillH\Documents\HexaBackups\newhexa_factory_backup.bin`
   plus a copy somewhere else (USB stick, OneDrive, etc.).

2. **Power off Gary (the K12 hexapod).**

3. **Connect USB-C from the ESP32 module on Gary to your PC.**

4. **Power on Gary.** Verify Arduino IDE detects the COM port.

5. **In Arduino IDE:**
   - Open `gary_v1.ino` (this will open the whole folder as a sketch)
   - Tools → Board → ESP32 Arduino → ESP32 Dev Module
     (or whatever specific board profile Hiwonder's docs recommend)
   - Tools → Port → select the COM port (CH340)
   - Click **Verify** first to compile without flashing — make sure it builds clean
   - If it builds, click **Upload** to flash

6. **After flashing**, open Tools → Serial Monitor at 115200 baud to see
   debug output as Gary boots and changes states.

## What to expect on first boot

If everything works:

1. Eyes flash white (BOOT)
2. Brief whistle sound plays (or silent if you haven't reflashed WonderEcho)
3. Eyes settle to slow-pulsing blue (IDLE)
4. Gary stands in self-balance mode

Try saying "Hello Hiwonder" → eyes go bright blue (LISTENING). If you say
"go forward" Gary should walk two steps forward and return to IDLE.

If you don't say anything for 30 seconds while in IDLE, Gary will start
ROAMING — eye color cycles purple/teal, ultrasonic-driven obstacle avoidance
kicks in. Say "stop" to bring him back.

If something loud happens (clap, slam) → STARTLED. Eyes flash red, jumps
back, recovers after 3 seconds.

If you tip Gary over → ALARMED. Slow red pulse, sad sound. Stays alarmed
until you set him level again.

## Tuning

Almost everything is in `gary_config.h`. Common tweaks:

- `SOUND_STARTLE_THRESHOLD` — lower if Gary's not startling enough,
  raise if he startles at every household noise
- `IDLE_TO_ROAM_MS` — how long Gary waits in IDLE before wandering
- `ROAM_BOREDOM_MS` — how often Gary perturbs his roam heading
- `WALK_SPEED` / `TURN_SPEED` — overall pace
- Any `EYE_*` color — RGB values for emotional states
- Feature flags at the bottom — turn things off/on for testing

## Debugging

`ENABLE_DEBUG_SERIAL = 1` (default) prints state changes and events
to the serial monitor. Useful for understanding what Gary is "thinking."

Watch the serial output to see:
- State transitions
- Voice command IDs
- Battery readings
- Tip detection events

## Known limitations of v1

- **Voice playback is a no-op** until you reflash WonderEcho with custom
  R2-D2 clips (see WonderEcho documentation + the gary_voice/ outputs folder).
  Set `ENABLE_VOICE_PLAYBACK = 1` after that.

- **Camera face-tracking is disabled** until you swap touch sensor for
  camera on the I2C bus. Set `ENABLE_CAMERA = 1` after that.
  "Come here" command won't work until camera is enabled.

- **Voice command IDs in `gary_config.h` are placeholders** matching
  Hiwonder's default WonderEcho firmware. If you customize WonderEcho's
  command vocabulary, update those IDs.

- **No LiDAR, no IR sensors, no cliff detection.** Reactive ultrasonic
  + IMU + voice + camera only.

- **VsenseMod-based gait feedback** (the K12-specific feature) is not
  used by this firmware. The v1.1-targeted Hiwonder libraries don't know
  about VsenseMod, so we get the open-loop gait. Walking should still
  work fine on the K12, just without closed-loop stability tweaks.

## Recovery

To go back to factory firmware:

```
python -m esptool --port COM14 write-flash 0 C:\Users\BillH\Documents\HexaBackups\newhexa_factory_backup.bin
```

(Substitute the actual COM port if it differs.)

## Honest assessment

This firmware is **untested.** It compiled in my head, not in Arduino IDE.
First flash will likely surface compile errors, runtime bugs, or behavior
surprises. That's normal and expected for v1 firmware.

Things most likely to need fixing:
- Library include paths (might need to be adjusted for your Arduino setup)
- The `gary.balance(true)` call — header showed `void balance(bool state)`
  but the example sketch called it as `gary.balance()`. May need adjustment.
- IMU euler angle ranges — assumed degrees; might be radians
- Voice command IDs — may not match what's actually in WonderEcho's factory firmware
- The `face_data_receive` 4-byte format — assumed [x, y, w, h];
  may be a different layout

Plan to iterate.
