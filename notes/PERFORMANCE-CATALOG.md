# GARY'S PERFORMANCE CATALOG (working draft, 2026-08-22)

Bill's commission: "mock-up what all of his standard actions /
call-response / additional actions are, so we can tune his performances
and possibly write them into personality traits — actions based on
triggers — watch them as we tune them, then integrate them into his
autonomous circuit."

Plan: (1) this list, (2) a PERFORMANCE GYM (door_gym-style --watch reps
per action for tuning), (3) trigger->action personality tables wired
into STATE_EXPLORE.

## A. Firmware scripted actions (GARY_v3 dispatch; voice command / PS5)

| action | what it does today | current trigger | proposed autonomous trigger |
|---|---|---|---|
| SAY_HELLO | greeting routine | "hello" | person detected (CSI/sound) |
| INTRODUCE | intro routine + speech | "introduce yourself" | first boot of the day |
| SHOW_SKILL | skill demo routine | "show a skill" | idle + audience (sound activity) |
| DANCE | dance routine | "dance" / PS5 | proud moment (task complete) |
| BOW | deep bow: 28 deg pitch + body drop, hold, unhurried rise | "bow" | after a performance; meeting Bill |
| WAVE | tall stand, 23 deg pitch back, big side-to-side rolls | "wave hand" | person leaves / arrives |
| MARCH_PLACE | march + determined growl + playful chatter | "march in place" | impatience (long wait at a door) |
| STRUT | hip-swing steps, head up | "strut" | after a BREACH victory |
| RUN (ACTION_1) | sprint + excited screaming | "execute action one" | ZOOMIES gate (open runway) |
| SAY_GOODBYE (ACTION_3) | sad chirp + back away | voice | shutdown ritual |
| SHOW DISTANCE (ACTION_4) | sonar bar-graph display | voice | (diagnostic, keep manual) |
| GOOD_JOB / HIGH_FIVE / HUG (5-7) | interaction routines | voice | praise detected |
| SIC_EM / CHARGE (8-9) | aggressive advance | voice | (manual only -- doctrine) |
| COME_HERE (11) | approach caller | voice | summons |
| EXPLORE (12) | STATE_EXPLORE | voice/console | the autonomous circuit itself |
| SENSING (13) | CSI hold-still capture | voice | scheduled sentry |
| PUSH_UP / SIT_UP / TWIST / SPREAD / FALL_STAND | calisthenics set | voice | morning warm-up ritual |
| LEFT/RIGHT HOOK | strike animations | voice | (performance only) |
| SHAKE_HEAD | look-around | voice | confusion (failed recognition) |
| SHAKE_OFF | body shake | voice | after rain / after a fall |
| COMBAT/FIGHT | stance | voice | (manual) |

## B. Voice library (gary_sounds, April; all firing rules NEW as of 2ce6305)

chirp_happy, chirp_snappy (duck), whistle_curious (gait change),
whistle_excited, chatter_playful, proud_fanfare, unsure_descend,
concerned_low (stall), sad_descend (3rd stall = DEFEAT), scream
(startle), shocked_blast, determined_growl (climb commit), laughter,
processing, whistle, acknowledge, query (wall), alert.
Tuning question per track: trigger, cooldown, and personality weight.

## C. Sim-certified choreography verbs (this week; field-portable)

| verb | source | trigger today |
|---|---|---|
| THE BREACH (mounts+bumps) | BreachFSM | shut door, kick failed |
| look-it-up-and-down | FSM ph 40 | 2 failed mounts |
| probe-knock | FSM ph 41 | third act; ajar entry |
| head-shake (true body_yaw wag) | FSM ph 42 | pre-mule defeat beat |
| turn-away + MULE-KICK | FSM ph 43/61/62 | last resort |
| spin-and-RUSH | FSM ph 9 rev | mule-kick victory |
| sad walk-off (defeat noise) | FSM ph 45 | tragedy end |
| shoulder sweep | FSM ph 93 | ajar door beside him |
| square-up + station | FSM ph 92 | pre-mount, pre-shove |
| UNTANGLE (tall backward climb-off) | nav deadman x2 | hung on geometry |
| basin-break wheel-away | nav deadman x3 | planning attractor |
| get-up reflex (1.44 s) | door_gym getup | belly-down |
| gimbal scan / crouch peek | nav | mapping ritual |
| wall mount + ascent draft | door_gym rear/wall | the rung-4 mission |

## D. Personality integration sketch (the later block)

- Trigger table: EVENT -> (action, probability, cooldown, mood-weight).
- Mood state: confidence (victories raise, defeats lower), energy
  (battery), patience (stall count). Mood scales which performance
  fires: high confidence + victory -> STRUT + proud_fanfare; low
  confidence + door -> extra probe-knocks before committing.
- The performance gym: watch each verb on a rep loop (existing --watch
  machinery), tune constants, Bill directs like the breach nights.
- Integration: STATE_EXPLORE gains the trigger table; firmware actions
  invoked via the existing dispatcher; sim mirrors via drv choreography.
