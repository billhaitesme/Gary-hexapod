# Gary — WonderEcho Configurator Paste Sheet

**Wake word:** `Hey Gary`
**Wake response clip:** `look.wav`

---

## Commands

Format: each row = one command. Primary phrase first, aliases follow.
For each command in the configurator: create the command, paste primary phrase, then add each alias as an additional phrase mapped to the same command ID.

### Movement

| # | Command ID name | Primary phrase | Aliases |
|---|---|---|---|
| 1 | FORWARD | go forward | forward; let's go |
| 2 | BACKWARD | go backward | get back; backward |
| 3 | TURN_LEFT | turn left | — |
| 4 | TURN_RIGHT | turn right | — |
| 5 | TURN_AROUND | turn around | — |
| 6 | GO_FASTER | go faster | — |
| 7 | SLOW_DOWN | slow down | — |
| 8 | STOP | stop | halt |
| 9 | RUN | run | — |

### Position / State

| # | Command ID name | Primary phrase | Aliases |
|---|---|---|---|
| 10 | SIT | sit | — |
| 11 | STAY | stay | — |
| 12 | WAKE_UP | wake up | — |
| 13 | GO_HOME | go home | — |

### Social

| # | Command ID name | Primary phrase | Aliases |
|---|---|---|---|
| 14 | SAY_HELLO | say hello | wave |
| 15 | SAY_GOODBYE | say goodbye | — |
| 16 | SAY_GOODNIGHT | say goodnight | — |
| 17 | GOOD_JOB | good job | good Gary |
| 18 | SHAKE | shake | — |
| 19 | HIGH_FIVE | high five | — |
| 20 | HUG | hug it out | — |
| 21 | DANCE | dance | — |
| 22 | SHOW_OFF | show off | — |

### Combat / Energy

| # | Command ID name | Primary phrase | Aliases |
|---|---|---|---|
| 23 | FIGHT | fight | — |
| 24 | SIC_EM | sic em | sick em; get em |
| 25 | CHARGE | charge | — |

### Dismissal

| # | Command ID name | Primary phrase | Aliases |
|---|---|---|---|
| 26 | GET_OUT | get out of here | get lost; go away |

### Approach

| # | Command ID name | Primary phrase | Aliases |
|---|---|---|---|
| 27 | COME_HERE | come here | — |

### Exploration

| # | Command ID name | Primary phrase | Aliases |
|---|---|---|---|
| 28 | EXPLORE | explore | go explore; wander |
| 29 | LOOK_AROUND | look around | — |

---

## Counts

- 29 commands total
- ~40 phrases including aliases
- Well under WonderEcho's 300-phrase capacity

---

## TTS Placeholder Strategy (Phase 1)

For each command's TTS response in the configurator, just type the command name as a short placeholder (e.g. `"forward"`, `"sit"`, `"shake"`). These get **replaced** with R2-D2 WAVs in Phase 2 — what matters now is that every command HAS a TTS slot so there's a file to swap.

---

## Track These as You Go

After submitting the config, WonderEcho assigns numeric IDs to each command. **Write down the actual ID it gives you for each command** — these go into `gary_config.h` in Phase 5.

Suggested format to record:

```
FORWARD     = ___
BACKWARD    = ___
TURN_LEFT   = ___
TURN_RIGHT  = ___
TURN_AROUND = ___
GO_FASTER   = ___
SLOW_DOWN   = ___
STOP        = ___
RUN         = ___
SIT         = ___
STAY        = ___
WAKE_UP     = ___
GO_HOME     = ___
SAY_HELLO   = ___
SAY_GOODBYE = ___
SAY_GOODNIGHT = ___
GOOD_JOB    = ___
SHAKE       = ___
HIGH_FIVE   = ___
HUG         = ___
DANCE       = ___
SHOW_OFF    = ___
FIGHT       = ___
SIC_EM      = ___
CHARGE      = ___
GET_OUT     = ___
COME_HERE   = ___
EXPLORE     = ___
LOOK_AROUND = ___
```

---

## Ambiguities

All resolved. Ready to paste.
