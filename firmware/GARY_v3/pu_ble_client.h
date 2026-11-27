#ifndef PU_BLE_CLIENT_H
#define PU_BLE_CLIENT_H

#ifdef __cplusplus
extern "C" {
#endif

// Call once from setup(), after BP32.setup(...). This registers the BTstack
// HCI/GATT handlers and automatically starts scanning for PU.
void pu_ble_init(void);

// Manually (re)trigger a connect attempt. Not required for normal operation
// since pu_ble_init() auto-starts scanning and disconnects auto-retry, but
// available if you want to force a retry from application code.
void pu_ble_request_connect(void);

// True once connected to PU and the writable characteristic has been found.
bool pu_ble_is_ready(void);

// Low-level: send a raw line. A trailing '\n' is appended automatically if
// missing. Returns false if not ready or if still within the send cooldown
// (caller should just try again next tick with a fresh value — this is not
// a queue, it always sends whatever is "now").
bool pu_send_line(const char* line);

// Typed helpers matching PU's wire protocol (T/S/B/P/R/X + value + '\n').
bool pu_send_turn(float value);
bool pu_send_speed(float value);
bool pu_send_button(int value);
bool pu_send_pitch(float value);
bool pu_send_roll(float value);
bool pu_send_text(const char* text);

// Convenience for joystick driving: alternates which of turn/speed gets the
// send slot each call, so one axis can't permanently starve the other under
// the send cooldown. Call this every tick with the current stick values
// while in PU relay mode.
bool pu_send_drive(float turn, float speed);

// DEPRECATED: was a bandwidth-saving toggle that only ever sent Turn/Speed
// OR Pitch/Roll, never both -- meaning head-look went completely silent
// during driving instead of just naturally doing nothing while walking (the
// real behavior joystick() already has built in). Left here in case
// anything still references it, but pu_send_all_axes() below is what
// actually matches how the original radio remote sent commands and should
// be used instead.
bool pu_send_drive_or_look(float turn, float speed, float lookPitch, float lookRoll);

// DEPRECATED: sent Turn, Speed, Pitch, and Roll independently, one per call
// in a fixed rotation (T, S, P, R, T, S, P, R, ...). Each axis paid its own
// PU_MIN_SEND_INTERVAL_MS cooldown, so any single axis (e.g. Pitch) only
// actually refreshed once every 4 cooldown periods -- ~1000ms in the
// worst case at the 250ms cooldown, which read as sluggish head-look.
// pu_send_all_axes_combined() below replaces this.
bool pu_send_all_axes(float turn, float speed, float pitch, float roll);

// Sends Turn, Speed, Pitch, and Roll together as one UART line ("A<turn>,
// <speed>,<pitch>,<roll>\n") instead of 4 separate ones, so all four axes
// pay the send cooldown exactly once per update instead of each paying it
// separately -- roughly a 4x cut in how often any given axis (esp.
// Pitch/Roll for head-look) actually gets fresh data to PU. PU parses this
// with a comma-scanning float reader (see the 'A' case in
// enableRemoteControlWithGroup() in pu_robot_extension_bluetooth.ts) that
// avoids allocating substrings, matching the existing charCodeAt()-based
// parser's GC-conscious style. Call this every tick with the current stick
// + tilt values while in PU relay mode.
bool pu_send_all_axes_combined(float turn, float speed, float pitch, float roll);

#ifdef __cplusplus
}
#endif

#endif // PU_BLE_CLIENT_H
