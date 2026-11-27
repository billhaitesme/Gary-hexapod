# Gary — Voice Vocabulary Specification

This document is the worksheet for two upcoming sessions:
1. **WonderEcho reflash** at wonderecho.net (define wake word + commands, build firmware)
2. **Action group recording** in Hiwonder PC software (record physical motions for animated commands)

Once both are done, Gary's main firmware (`gary_v1.ino` / `gary_config.h`) gets a final pass to map the actual command IDs WonderEcho assigns to behaviors.

---

## Wake Word

**"Hey Gary"** (3 syllables)

WonderEcho works best with 2-4 syllable wake words. "Hey Gary" hits the sweet spot — not so short it false-triggers from "hey" alone, not so long it's annoying to say.

Wake-word response on detection: plays `look.wav` (short attentive chirp).

---

## Command Vocabulary

Commands grouped by category. **Bold** = primary phrase. *Italic* = aliases (alternative phrasings that map to the same command).

### Movement

| Command | Phrases | Behavior |
|---|---|---|
| FORWARD | **"go forward"**, *"forward"*, *"let's go"* | Walk forward 2 steps |
| BACKWARD | **"go back"**, *"backward"* | Walk backward 2 steps |
| TURN_LEFT | **"turn left"** | Pivot left in place |
| TURN_RIGHT | **"turn right"** | Pivot right in place |
| TURN_AROUND | **"turn around"** | Full 180° spin |
| GO_FASTER | **"go faster"** | Increase WALK_SPEED for next motion |
| SLOW_DOWN | **"slow down"** | Decrease WALK_SPEED |
| STOP | **"stop"**, *"halt"* | Halt motion immediately, stay in LISTENING |
| RUN | **"run"** | Fast forward burst |

### Position / State

| Command | Phrases | Behavior |
|---|---|---|
| SIT | **"sit"** | Smooth lower body to ground via action group |
| STAY | **"stay"** | Freeze pose, balance off, override silence timeout |
| WAKE_UP | **"wake up"** | `wake_up()` animation; also serves as soft-reset |
| GO_HOME | **"go home"** | v1: stop + sit in current spot. v2+: navigate to home marker. |

### Social

| Command | Phrases | Animation | Sound |
|---|---|---|---|
| SAY_HELLO | **"say hello"**, *"wave"* | Wave (action group) | `look.wav` or `playful.wav` |
| SAY_GOODBYE | **"say goodbye"** | Wave (slower) | `sad.wav` |
| SAY_GOODNIGHT | **"say goodnight"** | Wave + sit, dim eyes | `whistle.wav` (slow) |
| GOOD_JOB | **"good job"**, *"good Gary"* | Proud bobble (acting_cute or new group) | `proud.wav` or `very_excited.wav` |
| SHAKE | **"shake"** | Handshake leg motion (action group) | `snappy.wav` |
| HIGH_FIVE | **"high five"** | Raise front leg (action group) | `very_excited.wav` |
| HUG | **"give us a hug"** | Front legs close around (action group) | `proud.wav` |
| DANCE | **"dance"** | Twist + leg waves (action group + built-in twist) | `whistling_excited.wav` |
| SHOW_OFF | **"show off"** | Sequence of impressive moves | `laughter.wav` |

### Combat / Energy

| Command | Phrases | Behavior |
|---|---|---|
| FIGHT | **"fight"** | Red eyes + pitched-forward prep stance, hold |
| SIC_EM | **"sic em"**, *"get em"* | Multi-stage: prep → charge → attack action group |
| CHARGE | **"charge"** | Bouncy forward run with `scream.wav` battle cry |

**Sic em sequence detail (multi-stage):**
1. **Prep (~1s):** eyes red, body pitches forward (att.x = -15°), hold
2. **Charge:** forward run at high speed
3. **Strike when in range:** trigger attack action group (front legs swing/poke motion)
4. **Recovery:** return to normal stance, back to LISTENING

### Dismissal

| Command | Phrases | Behavior |
|---|---|---|
| GET_OUT | **"get out of here"**, *"get lost"*, *"go away"* | Concerned chirp + 180° spin + return to ROAMING |

### Approach (camera-dependent, v1 placeholder)

| Command | Phrases | Behavior |
|---|---|---|
| COME_HERE | **"come here"** | Face-tracking approach (when ENABLE_CAMERA = 1) |
| FOLLOW_ME | **"follow me"** | *Deferred to v1.5+* |

### Exploration

| Command | Phrases | Behavior |
|---|---|---|
| EXPLORE | **"explore"**, *"go explore"*, *"wander"* | Manual ROAMING trigger |
| LOOK_AROUND | **"look around"** | Slow rotate scanning while staying in place |

---

## Total Phrase Count

Roughly **30 unique commands**, **~45 phrases** including aliases. WonderEcho holds 300 phrase capacity, so we're using ~15% — lots of room for v1.5/v2 additions.

---

## Action Groups Required

These are physical motion sequences that need to be recorded in Hiwonder PC software (Action Edit mode → drag servo sliders → keyframes → Download). Slot numbers are suggestions; actual IDs are assigned in PC software.

| Slot | Name | Used By |
|---|---|---|
| 1 | sit | SIT command, also part of SAY_GOODNIGHT |
| 2 | wave | SAY_HELLO, SAY_GOODBYE |
| 3 | shake | SHAKE command |
| 4 | high_five | HIGH_FIVE command |
| 5 | hug | HUG command |
| 6 | proud_bobble | GOOD_JOB command |
| 7 | show_off | SHOW_OFF command |
| 8 | dance | DANCE command |
| 9 | fight_stance | FIGHT command, prep phase of SIC_EM |
| 10 | attack_motion | strike phase of SIC_EM |
| 11 | stay_freeze | STAY command (just a held pose) |

**Hiwonder built-ins still in use:**
- `gary.acting_cute()` — used during curiosity moments and face detection
- `gary.wake_up()` — used for WAKE_UP command and BOOT recovery
- `gary.twist()` — used as part of DANCE

---

## Curiosity Behaviors (Autonomous, No Voice)

Gary performs these spontaneously during ROAMING — no voice command needed. Triggered by:

**Random timer (every 20-40s during ROAMING):**
- Pause + look around (slow yaw left/right)
- Sniff (head-down tilt + return + chirp)
- Investigation pose (raise body briefly, hold, settle)
- Random whistle/chatter chirp
- Quick spin

**Sensor-driven:**
- Color blob detected by camera → brief turn-toward + `whistle_inquisitive.wav` + resume
- Face detected → `acting_cute()` + resume
- Sub-startle sound spike → brief "huh?" head turn

These behaviors aren't bound to commands — they're how Gary feels alive between things you say to him.

---

## R2-D2 Sound Clip Mapping

The 17 R2-D2 WAVs already converted (in `gary_voice/wav_16khz/`) get mapped to WonderEcho phrase response slots during the configurator. Mapping by mood:

| Clip | Used For |
|---|---|
| look.wav | Wake word response, SAY_HELLO, ambient curious chirp |
| snappy.wav | Acknowledgment, SHAKE |
| unsure.wav | Confused / unrecognized command |
| processing.wav | Extended task running |
| chatter.wav | Ambient roaming chirp, ROAMING entry |
| whistling_excited.wav | DANCE, COME_HERE |
| proud.wav | GOOD_JOB, HUG, COMING ARRIVED |
| scream.wav | CHARGE battle cry, STARTLED |
| sad.wav | ALARMED, SAY_GOODBYE |
| whistle.wav | Boot startup, SAY_GOODNIGHT, ambient |
| playful.wav | Ambient roaming, alternate hello |
| very_excited.wav | HIGH_FIVE, big positive reactions |
| laughter.wav | SHOW_OFF, particularly fun moments |
| concerned.wav | GET_OUT dismissal, stuck |
| shocked.wav | Unexpected events, big surprises |
| determined.wav | WAKE_UP, FIGHT prep |
| whistle_inquisitive.wav | Color/face spotted, "huh?" curiosity |

---

## Workflow Steps for Reflash Session

When you're ready to do the WonderEcho reflash:

### Phase 1 — wonderecho.net (online configurator)

1. Sign up / log in at wonderecho.net
2. Go to Language Model Development
3. Set wake word: **"Hey Gary"**
4. Add each command phrase from the tables above
   - Use the primary phrase as the canonical command
   - Add aliases as additional phrases mapping to the same command ID
5. Go to Text-to-Speech section
6. Define a short placeholder TTS response for each command (will be replaced with R2-D2 clips)
7. Submit the configuration
8. Wait for processing
9. Download the generated firmware files

### Phase 2 — Local file substitution

1. Extract Hiwonder's WonderEcho firmware tool (probably on USB drive or downloadable)
2. Locate `firmware\voice\src\` directory in the extracted tool
3. Note which TTS WAV files map to which command IDs (check filenames carefully)
4. Replace each TTS file with the corresponding R2-D2 clip from `gary_voice/wav_16khz/`
   - Use the mapping table above
5. Verify all WAVs are 16kHz mono 16-bit PCM (the format we already converted to)

### Phase 3 — Build and flash WonderEcho

1. Run `Combine bin.bat` in the firmware tool directory
2. Run `Upgrade.bat` → Auto-allocate → Packaging
3. Save the resulting `.bin` file
4. **Disconnect WonderEcho's I2C cable from the hexapod**
5. Connect WonderEcho directly to PC via its USB-C port
6. Run `PACK_UPDATE_TOOL.exe`
7. Select the `.bin` file
8. Flash — watch for success message
9. Disconnect USB-C, reconnect I2C cable to hexapod

### Phase 4 — Test wake word

1. Power on hexapod
2. Say **"Hey Gary"**
3. Should hear `look.wav` chirp response
4. Verify WonderEcho is engaging — without flashing Gary firmware yet, it just plays the response without doing anything

### Phase 5 — Update Gary's main firmware

1. In `gary_config.h`:
   - Replace `CMD_FORWARD = 1` etc. with the actual command IDs from your WonderEcho config
   - Replace clip ID placeholders with the actual mapping you used
   - Set `ENABLE_VOICE_PLAYBACK = 1`
2. Re-flash Gary
3. Test full pipeline: "Hey Gary, go forward" → walks → returns to LISTENING
4. Test other commands one by one

### Phase 6 — Action group recording

1. With Gary on a stand (so legs don't bear weight while you pose them)
2. Open Hiwonder PC software, Action Edit mode
3. For each action group in the table:
   - Drag servo sliders to first keyframe pose
   - Click "Add action"
   - Adjust to next keyframe
   - Add action again
   - Continue until motion is complete
   - Set Action Group ID
   - Click Download
4. Test by clicking Run in the PC software
5. Once tested, the IDs are referenceable from Gary code via `gary.action_group_run(id)`

### Phase 7 — Wire up action groups in Gary firmware

1. Update command handlers in `gary_v1.ino` to call action groups instead of (or in addition to) `move()`
2. Add multi-stage sic-em sequence
3. Add curiosity behaviors during ROAMING
4. Re-flash Gary
5. Test

---

## Risks and Recovery

**WonderEcho flash failure:**
- WonderEcho can be re-flashed indefinitely via `PACK_UPDATE_TOOL.exe`. There's no "lock" state.
- If a flash bricks it, re-flashing usually recovers.
- Worst case: Hiwonder ships a "factory recovery" firmware bin you can flash to restore defaults.

**Gary main firmware regression:**
- Factory backup at `C:\Users\BillH\Documents\HexaBackups\newhexa_factory_backup.bin`
- Restore via `python -m esptool --port COM14 write-flash 0x0 [path-to-backup]`

**Action group corruption:**
- Action groups stored in SPIFFS — restoring factory firmware may or may not preserve them
- If lost, re-record (tedious but recoverable)

---

## What Stays as-is in Gary's Firmware

These don't depend on the WonderEcho reflash:

- State machine (BOOT → ROAMING → LISTENING → STARTLED → ALARMED)
- Sensor reading and battery monitoring
- Eye color palette and state indicators
- Sound startle (works with on-board mic, not WonderEcho)
- Tip detection
- Wall avoidance via ultrasonic
- Heartbeat debug output
- Boot stand-up sequence

These are already working as of v1.6.

---

## Next Time (When Ready)

You have:
- ✅ The vocabulary spec (this document)
- ✅ R2-D2 WAV files prepared (`gary_voice/wav_16khz/`)
- ✅ Working Gary main firmware (`gary_v1.ino` / `gary_config.h`)
- ✅ Factory backup as safety net

You need:
- WonderEcho firmware tool (probably on USB drive or from Hiwonder's docs)
- 2-3 hours of focused time for the reflash workflow
- Another session for action group recording
- Final session to wire it all together in Gary firmware

When you're ready, start with **Phase 1** above and work through sequentially.
