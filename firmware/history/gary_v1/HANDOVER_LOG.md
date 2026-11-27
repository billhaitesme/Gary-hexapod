# Gary Project — Session Handover Log

**Session date:** 2026-04-27 (single extended session, ~12 hours)
**Status at end of session:** Gary v1.6 firmware deployed and working on K12 hexapod (when run on battery, not USB)

---

## TL;DR for the next session

You have a working robot named Gary. He boots, walks, avoids obstacles, listens for voice commands, returns to listening for follow-ups, drifts back to roaming after silence, and reacts to loud sounds. He's running custom firmware on a Hiwonder MiniHexa K12 v1.0.

The next major milestone is the WonderEcho reflash (custom wake word "Hey Gary" + custom command vocabulary + R2-D2 sound substitution). The full spec for this is in `GARY_VOICE_SPEC.md` — that document is the worksheet for the next session.

After WonderEcho is reflashed, action groups need to be recorded for animated commands (sit, shake, dance, attack motion, etc), and Gary's main firmware updated with the actual command IDs and clip IDs from the configured WonderEcho.

---

## Hardware state

### Robots

- **K12 v1.0 (the new one):** Currently running Gary v1.6 firmware. Confirmed working on battery. Servos healthy. Calibration intact (factory backup preserved it).
- **MiniHexa v1.1 (the old one, was bricked):** Successfully restored from K12's factory backup. Walks again. Has weak servos that should be replaced eventually. Currently has K12's calibration values which don't perfectly match v1.1 mechanical alignment — three legs don't fully touch ground when standing, but it functions. Ideal candidate for use as a development/test bed since flashing experimental code there carries less risk than on the daily-driver K12.

### Modules currently connected to K12 I2C bus

- LED matrix
- WonderEcho (still has factory firmware — wake word is "Hello Hiwonder")
- Touch sensor (mounted but to be swapped for camera when ready)

### Modules ready but not yet connected

- ESP32-S3 camera module (replaces touch when ready)
- IR obstacle sensors x2 (need mounting)
- VL53L0X ToF sensors x5 (need I2C splitter + addressing)
- D500 LiDAR (UART, complex integration)

### Dev tools

- Pi 5 in Pironman 5 case (sitting out v1, future co-processor)
- Arduino Uno R4 WiFi + SunFounder Elite Explorer kit (separate sandbox)

---

## Critical files and their locations

### On Bill's PC

- `C:\Users\BillH\Documents\Arduino\gary_v1\` — Gary's sketch folder (current version of all firmware files)
- `C:\Users\BillH\Documents\HexaBackups\newhexa_factory_backup.bin` — **factory firmware backup, 4MB exactly, 4194304 bytes.** This is the safety net. Verified working as both backup AND restore source for both K12 and v1.1.
- `C:\Users\BillH\Documents\MiniHexa\` — cloned Hiwonder GitHub repo with all example sketches and library files

### Outputs from this session

- `gary_v1/gary_v1.ino` — Gary v1.6 firmware (state machine, behaviors, debug)
- `gary_v1/gary_config.h` — all tunable values (timings, thresholds, eye colors, voice IDs, feature flags)
- `gary_v1/README.md` — original setup/flash/recovery instructions
- `gary_v1/GARY_VOICE_SPEC.md` — **vocabulary spec and reflash workflow worksheet (next session uses this)**
- `gary_voice/wav_16khz/` — 17 R2-D2 clips converted to 16kHz mono 16-bit PCM WAV (~830KB total)
- `gary_voice/wav_22khz/` — same clips at 22.05kHz (alternative if needed)
- `gary_voice/original_mp3s.zip` — source MP3s for reference

---

## What works (confirmed, observed in serial output)

- Compile and flash workflow via Arduino IDE → ESP32 Dev Module → COM14
- Boot sequence: white eyes → stand-up to working height (`pos.z = 1.5`) → ROAMING
- State machine transitions correctly: BOOT → ROAMING → LISTENING → ROAMING (and STARTLED, ALARMED as edge cases)
- Voice command recognition through WonderEcho factory firmware (verified `CMD: 4` for "turn right" was received and handled)
- LISTENING two-tier timeout: 5s window for first command after wake, then 90s extended window after each subsequent command
- Sound startle reactivity (only on loud noises now, threshold 3800/2500)
- Battery monitoring with sanity floor (5000mV) and 3-consecutive-low requirement before ALARMED triggers
- Distance reading from ultrasonic eyes (real values 200-700mm range, accurate)
- Eye color changes based on state
- `balance(true)` self-leveling
- Restore-from-backup process (proven twice)

## What doesn't work / is deferred

- **Custom wake word "Hey Gary"** — needs WonderEcho reflash (next major project per spec)
- **Custom commands like "get out of here", "sic em", "dance"** — needs WonderEcho reflash + action group recording
- **R2-D2 sound playback** — `ENABLE_VOICE_PLAYBACK = 0` until WonderEcho is reflashed with R2-D2 WAVs
- **Camera-based features (face tracking, color following, "come here")** — `ENABLE_CAMERA = 0` until camera is wired in (touch sensor needs to be swapped out, or splitter installed)
- **Boredom timer / heading perturbation** — `ENABLE_BOREDOM_TIMER = 0` because it was overriding `avoid()` and causing wall-bumping. Will reintroduce as smarter version that only kicks when path is clear.
- **Touch sensor reactions** — deferred until I2C splitter
- **IR obstacle sensors** — deferred (mounting + splitter)
- **VL53L0X cliff detection** — deferred (splitter + addressing)
- **LiDAR integration** — deferred (UART situation, complexity)
- **Pi co-processor** — deferred to v2+
- **Person-specific face recognition** — deferred (needs Pi/embeddings)
- **PS5 DualSense controller** — deferred to its own session

---

## Critical bug discovered late in session: USB power instability

**The single most important finding tonight, learned in the last hour:**

When Gary is connected to PC via USB-C, the dual-power scenario (USB 5V + battery) causes:
- Spurious battery readings (3430mV when battery is actually 7800mV)
- Constant beeping from MM32 servo controller (its own low-voltage hardware alarm)
- Ground-loop noise picked up by on-board mic, causing false STARTLED transitions
- Generally unreliable behavior

When run on battery only (USB disconnected), Gary behaves correctly:
- Stable battery readings
- No beeping
- Sound startle only fires on truly loud events (loud claps)
- Walking and obstacle avoidance work as designed

**Practical implication for development:**
- Flash via USB → disconnect USB → power-cycle for testing → reconnect USB only for next flash
- Future improvement: add Bluetooth or WiFi serial bridge so we can debug while running on battery (the ESP32 has both, just not used yet)

**This finding rewrites several hours of mid-session debugging.** What we initially thought were software bugs (constant STARTLED firing, weird battery values, beeping) were artifacts of USB-induced power instability. Gary's actual code behavior on battery was always working correctly.

---

## Things future-Claude/future-Bill should NOT do

- **Don't blindly trust serial output during USB-tethered debugging.** Cross-check on battery before concluding Gary is misbehaving.
- **Don't write to deviation values via Hiwonder PC software's "Download offset" button** without first capturing what's already there. The "Read offset" button does something to servos as a side effect, not a passive read.
- **Don't restore factory firmware from K12's backup onto v1.1 without re-calibration.** The factory backup includes calibration data and writing it to a different physical robot gives that robot the wrong trim values.
- **Don't use action groups for** any cases where the user previously said "no action groups." (Note: user later relaxed this constraint for v1.x — action groups now permitted.)
- **Don't suggest taking breaks or stopping** unless user has explicitly indicated they're done. User stated this preference multiple times during the session and it should be respected.

---

## Tunable values in gary_config.h (current as of v1.6)

```cpp
// Timing
LOOP_DELAY_MS              50
LISTENING_FIRST_TIMEOUT_MS 5000   // 5s window for first command after wake
LISTENING_TIMEOUT_MS       90000  // 1.5min extended after a command
ROAM_BOREDOM_MS            8000   // (currently disabled feature)
ROAM_TIMEOUT_MS            300000 // 5min roam → IDLE (rarely reached)
DISMISSAL_TURN_MS          1500   // 180° turn for "get out of here"
COMING_SEARCH_TIMEOUT_MS   15000  // Face search give-up
STARTLED_DURATION_MS       3000   // Recover from startle

// Sensor thresholds
SOUND_STARTLE_THRESHOLD    3800   // Out of 4095 max — only big claps trigger
SOUND_STARTLE_DELTA        2500   // Big jump above ambient required
DISTANCE_OBSTACLE_MM       200    // (passed implicitly via avoid())
BATTERY_LOW_MV             6600   // 3 consecutive readings below this = ALARMED
IMU_TIP_THRESHOLD_DEG      30

// Movement
WALK_SPEED                 2.0f
TURN_SPEED                 1.5f
STEP_TIME_MS               1000
DEFAULT_STEPS              2

// Feature flags (current state)
ENABLE_VOICE               1
ENABLE_CAMERA              0   // Off until camera wired in
ENABLE_BALANCE             1
ENABLE_SOUND_STARTLE       1
ENABLE_BATTERY_MONITOR     1
ENABLE_BOREDOM_TIMER       0   // Off — was breaking avoid()
ENABLE_VOICE_PLAYBACK      0   // Off until WonderEcho reflashed
ENABLE_DEBUG_SERIAL        1
```

---

## Real sensor data observed (for tuning reference)

These are actual readings observed from Gary on battery, walking around:

- **Sound (12-bit ADC):** Range 0-4095. Walking quiet = 0-50. Walking with footfalls = 50-300. Occasional spike to 1500-2000 during heavy footfalls. Loud clap right next to him = 4095 (railed). Threshold 3800 only catches genuinely loud events.

- **Distance (ultrasonic, mm):** Range observed 200-22000mm. The 22000 is junk reading at boot before sensor stabilizes. After warmup, reads sensible 200-2000mm. Held distance from obstacle around 300-400mm during avoid().

- **Battery (mV):** Healthy fresh batteries = 7800-7900mV. After ~1 hour of testing = 7000-7200mV. Below 6800 starting to feel sag under load. Threshold 6600 is appropriate for "actually getting low" alarm.

- **IMU euler (degrees):** Roll and pitch read 0.0/0.0 consistently when level. May be due to Madgwick filter not having drift, or could be reading-format issue. Not false-tripping the 30° tilt threshold, so functional regardless.

---

## Hiwonder API gotchas discovered

- `move()` requires `pos.z = 1.5` (or similar non-zero) for proper standing height. Default of 0 produces tucked legs. This was the breakthrough fix early in firmware iteration.
- `balance(bool state)` — takes a bool argument, not no-args (despite some example sketches calling it with no args).
- `gary.board.bat_voltage` after `bat_voltage_update()` returns mV but with occasional spurious readings (3000-4000 range). Sanity floor needed.
- `gary.board.get_sound_val()` — raw 12-bit ADC reading 0-4095. Not millivolts, not normalized.
- `gary.sensor.set_ultrasound_rgb(mode, rgb1, rgb2)` — modes: `RGB_WORK_SOLID_MODE = 0`, `RGB_WORK_BREATHING_MODE = 1`.
- `gary.sensor.asr.rec_recognition()` — returns command ID from WonderEcho. Returns 0 when nothing recognized. Factory firmware command IDs: 1=forward, 2=backward, 3=turn_left, 4=turn_right, 9=stop, 29=walk_two_steps.
- WonderEcho I2C address 0x34 (CI1302 chip, 2MB flash).
- Camera I2C address 0x52 (ESP32-S3 onboard).
- Ultrasonic I2C address 0x77.
- Camera frame width is 160px (per `fmap` in color_tracking.ino).
- Camera detection returns 4 bytes: [x, y, w, h]. `val[0] != 0` = "saw something."

---

## Library file source notes

- All Hiwonder libraries live alongside each example sketch (per-folder copies, not a global library). Copy from any clean example folder.
- Recommended canonical source for library files: `C:\Users\BillH\Documents\MiniHexa\arduino-programming-projects\4.2 运动控制基本教程\5.2 全向运动\omnidirectional_movement\`
- Required files: `hiwonder_robot.{h,cpp}`, `hiwonder_board.{h,cpp}`, `hiwonder_servo.{h,cpp}`, `hiwonder_sensor.{h,cpp}`, `hiwonder_i2c.{h,cpp}`, `MadgwickAHRS.{h,cpp}`, `WMMatrixLed.{h,cpp}`, `global.h`
- `kinematics.h` is from a separate library (`Kinematics.zip`) installed via Arduino Library Manager (Sketch → Include Library → Add ZIP)
- The `library` library at `c:\Users\BillH\Documents\Arduino\libraries\library\src\esp32` is precompiled and contains ESP32-specific binaries — it's normal that the compiler reports "precompiled library" for it.

---

## Issues encountered and resolved

| Issue | Symptom | Cause | Fix |
|---|---|---|---|
| Tucked legs on first flash | Gary stood with legs collapsed inward, walked weirdly | `pos.z = 0` in code; IK solves for body-on-ground | Set `pos.z = 1.5` and add explicit stand-up in setup() |
| Constant STARTLED transitions | Gary cycled ROAMING ↔ STARTLED rapidly | USB ground-loop noise on mic input | Test on battery; raised thresholds to 3800/2500 |
| Wall bumping | Gary ran into walls during ROAMING | Boredom timer overrode `avoid()` mid-cycle | Disabled boredom timer (will reintroduce smarter) |
| Spurious low battery → ALARMED | Random ALARMED triggers | ADC noise during USB use producing 3000-4000mV readings | Sanity floor 5000mV + 3 consecutive low requirement |
| Beeping during USB testing | Continuous beeping that we attributed to firmware | MM32 hardware low-voltage alarm from USB instability | Disconnect USB during testing |
| Serial output flooded | Voltage telemetry from Hiwonder library spammed everything | Library prints `voltage: NNNN count:N` continuously | Live with it; our debug uses distinctive prefix `>>> GARY` |
| v1.4 changes seemed to not flash | New code didn't appear to take effect | (Ambiguous — possibly Arduino IDE caching, possibly USB issues) | Eventually resolved with v1.5 boot-flag print to verify what's compiled |

---

## Open questions for next session

1. **WonderEcho reflash workflow exact details:** Need to find Hiwonder's WonderEcho firmware tool (probably on USB or downloadable). Verify exact directory structure of `firmware\voice\src\`.

2. **WAV format for WonderEcho:** Spec says 16kHz mono 16-bit PCM. Verify this matches what the configurator generates. May need to convert to 22kHz if WonderEcho prefers higher sample rate.

3. **Action group recording:** Will need to put Gary on a stand (legs not bearing weight) and pose him through keyframes. May want to design poses on paper first before live recording.

4. **The "voltage 34298" noise:** Old library debug print we never identified the meaning of. Probably an internal library counter or scaled value. Doesn't impact Gary's operation but spams serial.

5. **IMU euler reading 0.0/0.0:** Stable but suspiciously perfect. Either Madgwick is well-tuned or readings aren't actually changing. Worth deeper investigation if we ever want tilt-based features.

6. **v1.1 calibration:** Old hexapod has K12's calibration values, three legs don't ground perfectly. To restore proper v1.1 calibration we'd need to either find an original v1.1 backup (don't have one) or run Hiwonder's deviation calibration program manually. User has had bad experiences with Hiwonder's calibration tools — proceed with caution.

---

## Bill's preferences (observed during session)

- Casual conversation tone, occasional profanity is fine.
- Doesn't want repeated suggestions to take breaks unless he says he's done.
- Appreciates honesty when Claude is wrong — explicit course-corrections preferred over hedging.
- Wants to verify before assuming (e.g., re-checking files rather than trusting earlier assumptions).
- Skill level: hobbyist developer, comfortable with PowerShell and Arduino IDE, some understanding of embedded concepts.
- Has previously bricked one hexapod by inexperienced flashing. Cautious about that.
- Doesn't want to repeat work already done in previous (unrelated) chats.
- Likes iterative refinement — "we can always change it later" is his philosophy.

## Style guidance for Claude in next session

- Short, direct messages preferred over long explanations.
- When recommending changes, be specific about what file/line/value to change.
- When uncertain, say so honestly rather than guessing.
- Provide actual files via `present_files` rather than telling Bill to copy/paste large code blocks.
- Don't reflexively suggest stopping or breaking. Bill will say when he's done.

---

## Recommended sequence for next session(s)

### Session A — WonderEcho reflash (focused, 2-3 hours)

Follow the workflow in `GARY_VOICE_SPEC.md` Phases 1-4. End state: WonderEcho responds to "Hey Gary" with a chirp.

### Session B — Action group recording (focused, 2-3 hours)

Per spec, record 11 action groups in Hiwonder PC software. Test each as it's recorded. End state: action groups stored on robot, callable by ID.

### Session C — Final firmware integration (focused, 1-2 hours)

Update gary_v1.ino to:
1. Use real command IDs from WonderEcho config
2. Use real clip IDs for sound playback
3. Wire up action group calls for animated commands
4. Implement multi-stage sic-em sequence
5. Add curiosity behaviors during ROAMING
6. Set ENABLE_VOICE_PLAYBACK = 1

End state: full Gary v2 with custom voice, custom commands, animated responses, and personality.

### Session D — Hardware expansion (longer)

Add I2C splitter. Wire camera. Update Gary firmware to use camera. Eventually add IR sensors, VL53L0X cliff detection. Each is a separate iteration.

---

## How to start the next chat

Suggested opener:

> Continuing the Gary hexapod project from a previous session. Attaching the voice spec, current Gary firmware files (gary_v1.ino, gary_config.h), and the handover log. Gary v1.6 is currently flashed and working. Today I want to start the WonderEcho reflash workflow per Phase 1 of the spec. Let's go.

Attachments to include:
- `GARY_VOICE_SPEC.md`
- `gary_v1.ino`
- `gary_config.h`
- `HANDOVER_LOG.md` (this document)

That gives the next instance everything needed to pick up smoothly.

---

*End of handover log.*
