// =============================================================================
// gary_sounds.h
//
// R2-D2-style emotional sound synthesis for Gary's buzzer.
//
// Each gary_*() call queues a "track" composed of FREQUENCY SWEEPS rather
// than held tones. A sweep linearly interpolates from start_hz to end_hz
// over a given duration, producing the warbles, chirps, and trills R2-D2
// is known for. A held tone is just a sweep where start == end.
//
// Calls return IMMEDIATELY. gary_sounds_update() must be called every loop
// iteration to advance the playback.
//
// Calling another gary_*() while a track plays INTERRUPTS the current track.
//
// Uses Hiwonder's pre-configured LEDC channel 0 on BUZZER_PIN=21.
// =============================================================================

#ifndef GARY_SOUNDS_H
#define GARY_SOUNDS_H

#include <stdint.h>

void gary_sounds_begin();
void gary_sounds_set_enabled(bool en);
void gary_sounds_update();
void gary_sounds_stop();
bool gary_sounds_is_playing();

// -----------------------------------------------------------------------------
// Track triggers — each returns immediately.
// -----------------------------------------------------------------------------

void gary_chirp_happy();          // Quick rising bleep — acknowledgment
void gary_chirp_snappy();         // Sharp double-blip — "got it"
void gary_whistle_curious();      // Slow ascending warble — inquisitive
void gary_whistle_excited();      // Fast rising trill — excited
void gary_chatter_playful();      // Random bleeps — ambient
void gary_proud_fanfare();        // Triumphant 3-tone with rising sweep
void gary_unsure_descend();       // Falling slow — confused
void gary_concerned_low();        // Low warble — worried
void gary_sad_descend();          // Long descending sweep — sad
void gary_scream();               // Erratic high warble + descend — startled
void gary_shocked_blast();        // Sharp high pulse — shocked
void gary_determined_growl();     // Low rising — fight mode
void gary_laughter();             // Chuckle pattern — happy
void gary_processing();           // Vibrato hum — thinking (no longer flat!)
void gary_whistle();              // Single mid sweep
void gary_acknowledge();          // Quick affirmative beep
void gary_query();                // Two-note "huh?"
void gary_alert();                // Insistent attention-getter

#endif // GARY_SOUNDS_H
