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

// Convenience for driving + head-look sharing one send slot: while the
// stick is off-center (|turn| or |speed| above deadzone), behaves exactly
// like pu_send_drive() -- alternates Turn/Speed. Once the stick is
// centered, uses that same slot to alternate Pitch/Roll from the
// controller's own tilt instead, restoring "tilt the controller to look
// around" while idle -- without adding any extra BLE traffic on top of
// driving. Call this every tick in place of pu_send_drive().
bool pu_send_drive_or_look(float turn, float speed, float lookPitch, float lookRoll);

#ifdef __cplusplus
}
#endif

#endif // PU_BLE_CLIENT_H
