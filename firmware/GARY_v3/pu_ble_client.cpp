// pu_ble_client.cpp
//
// BTstack GATT client for Gary (ESP32 / Bluepad32) to connect to PU's
// micro:bit Nordic UART Service (NUS) BLE peripheral and send it wire-
// protocol lines (T/S/B/P/R/X + value + '\n').
//
// Latest change: PU_MIN_SEND_INTERVAL_MS raised from 100 to 250.
//
// Diagnosis: Gary's own GATT layer is healthy throughout (state stays
// READY, ~90% write success, watchdog_recoveries always 0 — no stuck
// in-flight flag). But PU-side corner-pixel diagnostics (added to
// pu_robot_emotions.js) showed both receive-indicator LEDs going dark and
// staying dark exactly when PU stops responding, while the BLE connection
// itself never drops. That means PU's bluetooth.onUartDataReceived
// callback itself stops firing after a sustained burst of traffic — most
// likely the micro:bit's limited RAM getting exhausted/fragmented by the
// string allocations in that handler (uartReadUntil/charAt/substr) piling
// up faster than GC can keep up, since it always happens after volume, not
// on any particular bad message. Lowering Gary's send rate reduces total
// message throughput PU has to process per second, giving it more
// breathing room. Still well inside PU's own 400ms "stick still held"
// staleness window, so driving responsiveness shouldn't suffer.
//
// (Everything else below is unchanged from the last working version:
// write-without-response preferred per-characteristic; state-transition +
// watchdog diagnostics; explicit hci.h/gap.h/ble/gatt_client.h includes and
// btstack_packet_callback_registration_t, matching this BTstack port's
// actual headers.)

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <Arduino.h>

extern "C" {
#include "btstack.h"
#include "btstack_event.h"
#include "btstack_run_loop.h"
#include "hci.h"
#include "gap.h"
#include "ble/gatt_client.h"
}

#include "pu_ble_client.h"

#define PU_MIN_SEND_INTERVAL_MS   250
#define PU_STUCK_WATCHDOG_MS      1000
#define PU_RECONNECT_DELAY_MS     1500
#define PU_DEVICE_NAME_MATCH      "givev"
// Workaround for the unresolved BLE receive-stall bug: proactively cycle the
// connection on a fixed timer instead of waiting for it to hang. A fresh
// connection reliably works for a while, so this keeps PU from ever staying
// stuck for long even though the underlying cause isn't fixed.
#define PU_REFRESH_INTERVAL_MS    20000

static const uint8_t NUS_SERVICE_UUID[16] = {
    0x6E, 0x40, 0x00, 0x01, 0xB5, 0xA3, 0xF3, 0x93,
    0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E
};

typedef enum {
    PU_STATE_IDLE = 0,
    PU_STATE_SCANNING,
    PU_STATE_CONNECTING,
    PU_STATE_DISCOVER_SERVICE,
    PU_STATE_DISCOVER_CHAR,
    PU_STATE_READY
} pu_state_t;

static const char* pu_state_name(pu_state_t s) {
    switch (s) {
        case PU_STATE_IDLE:             return "IDLE";
        case PU_STATE_SCANNING:         return "SCANNING";
        case PU_STATE_CONNECTING:       return "CONNECTING";
        case PU_STATE_DISCOVER_SERVICE: return "DISCOVER_SERVICE";
        case PU_STATE_DISCOVER_CHAR:    return "DISCOVER_CHAR";
        case PU_STATE_READY:            return "READY";
        default:                        return "?";
    }
}

static pu_state_t pu_state = PU_STATE_IDLE;

static void pu_set_state(pu_state_t new_state) {
    if (new_state != pu_state) {
        Serial.printf("[PU-BLE] state: %s -> %s\n", pu_state_name(pu_state), pu_state_name(new_state));
        pu_state = new_state;
    }
}

static hci_con_handle_t pu_connection_handle = HCI_CON_HANDLE_INVALID;
static uint16_t pu_rx_value_handle = 0;
static bool pu_rx_write_without_response = true;

static bd_addr_t pu_addr;
static bd_addr_type_t pu_addr_type;
static bool pu_addr_known = false;

static gatt_client_service_t pu_service;
static uint16_t pu_service_start_handle = 0;

static btstack_packet_callback_registration_t pu_hci_event_callback_registration;

static btstack_context_callback_registration_t pu_scan_callback_reg;
static btstack_context_callback_registration_t pu_connect_callback_reg;
static btstack_context_callback_registration_t pu_send_callback_reg;
static btstack_context_callback_registration_t pu_can_write_callback_reg;

static btstack_timer_source_t pu_send_cooldown_timer;
static btstack_timer_source_t pu_reconnect_timer;
static btstack_timer_source_t pu_refresh_timer;

static char pu_send_buf[64];
static bool pu_send_in_flight = false;
static uint32_t pu_send_in_flight_since_ms = 0;
static bool pu_drive_toggle = false;

static uint32_t pu_diag_send_attempts = 0;
static uint32_t pu_diag_send_errors = 0;
// Per-axis send counters. The "last=" field in the diag line only ever
// shows whatever's in pu_send_buf at diag-print time, and pu_maybe_print_diag
// fires every 2000ms -- an exact multiple of the ~1000ms T/S/P/R rotation
// period in pu_send_all_axes(). That's a strobe: "last=" will alias onto the
// same axis every single print regardless of whether all four are actually
// being sent, making it look like only one axis is going out. These
// counters aren't subject to that and show the true distribution.
static uint32_t pu_diag_turn_sent = 0;
static uint32_t pu_diag_speed_sent = 0;
static uint32_t pu_diag_pitch_sent = 0;
static uint32_t pu_diag_roll_sent = 0;
static uint32_t pu_diag_total_hci_events = 0;
static uint32_t pu_diag_adv_reports = 0;
static uint32_t pu_diag_watchdog_recoveries = 0;
static uint32_t pu_diag_last_print_ms = 0;

static void pu_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void pu_gatt_client_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void pu_send_cooldown_timer_handler(btstack_timer_source_t *ts);
static void pu_reconnect_timer_handler(btstack_timer_source_t *ts);
static void pu_refresh_timer_handler(btstack_timer_source_t *ts);
static void pu_do_start_scan(void *context);
static void pu_do_connect(void *context);
static void pu_do_send(void *context);
static void pu_write_now(void *context);
static void pu_maybe_print_diag(void);

static inline uint32_t pu_now_ms() { return millis(); }

static bool pu_adv_name_matches(const uint8_t *adv_data, uint8_t adv_len) {
    int i = 0;
    while (i + 1 < adv_len) {
        uint8_t field_len = adv_data[i];
        if (field_len == 0) break;
        uint8_t field_type = adv_data[i + 1];
        if (field_type == 0x08 || field_type == 0x09) {
            const uint8_t *name = &adv_data[i + 2];
            uint8_t name_len = field_len - 1;
            char namebuf[32];
            uint8_t copy_len = name_len < sizeof(namebuf) - 1 ? name_len : sizeof(namebuf) - 1;
            memcpy(namebuf, name, copy_len);
            namebuf[copy_len] = 0;
            if (strstr(namebuf, PU_DEVICE_NAME_MATCH) != NULL) {
                Serial.printf("[PU-BLE] adv name match: '%s'\n", namebuf);
                return true;
            }
        }
        i += field_len + 1;
    }
    return false;
}

static void pu_do_start_scan(void *context) {
    (void) context;
    // Whatever triggered this scan (the reconnect timer firing, or a manual
    // pu_ble_request_connect() from Gary's CTRL code beating it there), a
    // fresh scan/connect cycle is starting right now, so any still-pending
    // reconnect timer from an earlier cycle is no longer needed. Removing it
    // here (safe no-op if it's not currently in the list) prevents the timer
    // ever being added twice further down the line.
    btstack_run_loop_remove_timer(&pu_reconnect_timer);
    pu_addr_known = false;
    pu_set_state(PU_STATE_SCANNING);
    gap_set_scan_parameters(1, 0x0030, 0x0030);
    gap_start_scan();
    Serial.println("[PU-BLE] Scanning...");
}

static void pu_do_connect(void *context) {
    (void) context;
    if (!pu_addr_known) {
        pu_set_state(PU_STATE_IDLE);
        return;
    }
    gap_connect(pu_addr, pu_addr_type);
}

static void pu_reconnect_timer_handler(btstack_timer_source_t *ts) {
    (void) ts;
    if (pu_state == PU_STATE_IDLE) {
        pu_do_start_scan(NULL);
    }
}

static void pu_schedule_reconnect(void) {
    // ROOT CAUSE of the "assert failed: btstack_run_loop_base_add_timer" /
    // reboot loop: this timer got re-added while a previous instance of the
    // same pu_reconnect_timer was still linked into BTstack's internal timer
    // list (i.e. a second disconnect fired -- and this function got called
    // again -- before the first 1500ms reconnect timer had fired/self-
    // removed). BTstack's linked list can't hold the same node twice; adding
    // it again corrupts the list, so it asserts and halts rather than
    // silently break. Confirmed reproducible: happens whenever disconnects
    // cascade faster than PU_RECONNECT_DELAY_MS, which the new periodic
    // refresh timer (plus PU's own reconnect instability) makes much more
    // likely to hit than before. Removing the timer first (safe no-op if
    // it's not currently in the list) makes this call idempotent no matter
    // how often or how close together it's invoked.
    btstack_run_loop_remove_timer(&pu_reconnect_timer);
    btstack_run_loop_set_timer_handler(&pu_reconnect_timer, &pu_reconnect_timer_handler);
    btstack_run_loop_set_timer(&pu_reconnect_timer, PU_RECONNECT_DELAY_MS);
    btstack_run_loop_add_timer(&pu_reconnect_timer);
}

// Periodic connection-cycling workaround: while READY, force a disconnect
// every PU_REFRESH_INTERVAL_MS. The existing HCI_EVENT_DISCONNECTION_COMPLETE
// handler + pu_schedule_reconnect() bring up a fresh connection automatically,
// which reliably works for a while even though the underlying stall cause
// hasn't been root-caused yet.
static void pu_refresh_timer_handler(btstack_timer_source_t *ts) {
    (void) ts;
    if (pu_state == PU_STATE_READY && pu_connection_handle != HCI_CON_HANDLE_INVALID) {
        Serial.println("[PU-BLE] periodic refresh: cycling connection to avoid stall.");
        gap_disconnect(pu_connection_handle);
    }
}

static void pu_arm_refresh_timer(void) {
    // Safe even if not currently registered; avoids corrupting the timer
    // list if this gets called again before the previous timer fired.
    btstack_run_loop_remove_timer(&pu_refresh_timer);
    btstack_run_loop_set_timer_handler(&pu_refresh_timer, &pu_refresh_timer_handler);
    btstack_run_loop_set_timer(&pu_refresh_timer, PU_REFRESH_INTERVAL_MS);
    btstack_run_loop_add_timer(&pu_refresh_timer);
}

static void pu_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void) channel; (void) size;
    if (packet_type != HCI_EVENT_PACKET) return;
    pu_diag_total_hci_events++;

    uint8_t event_type = hci_event_packet_get_type(packet);

    switch (event_type) {
        case GAP_EVENT_ADVERTISING_REPORT: {
            if (pu_state != PU_STATE_SCANNING) break;
            pu_diag_adv_reports++;

            const uint8_t *adv_data = gap_event_advertising_report_get_data(packet);
            uint8_t adv_len = gap_event_advertising_report_get_data_length(packet);

            if (pu_adv_name_matches(adv_data, adv_len)) {
                gap_event_advertising_report_get_address(packet, pu_addr);
                pu_addr_type = (bd_addr_type_t) gap_event_advertising_report_get_address_type(packet);
                pu_addr_known = true;

                gap_stop_scan();
                pu_set_state(PU_STATE_CONNECTING);
                Serial.println("[PU-BLE] Connecting to PU...");

                pu_connect_callback_reg.callback = &pu_do_connect;
                pu_connect_callback_reg.context = NULL;
                btstack_run_loop_execute_on_main_thread(&pu_connect_callback_reg);
            }
            break;
        }

        case HCI_EVENT_LE_META:
            if (hci_event_le_meta_get_subevent_code(packet) == HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
                if (pu_state != PU_STATE_CONNECTING) break;

                uint8_t status = hci_subevent_le_connection_complete_get_status(packet);
                if (status != ERROR_CODE_SUCCESS) {
                    Serial.printf("[PU-BLE] connection failed, status=%u. Retrying scan.\n", status);
                    pu_set_state(PU_STATE_IDLE);
                    pu_schedule_reconnect();
                    break;
                }

                pu_connection_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                Serial.printf("[PU-BLE] Connected, handle=%u. Discovering NUS service...\n", pu_connection_handle);
                pu_set_state(PU_STATE_DISCOVER_SERVICE);
                pu_service_start_handle = 0;
                pu_rx_value_handle = 0;
                gatt_client_discover_primary_services_by_uuid128(&pu_gatt_client_event_handler, pu_connection_handle, NUS_SERVICE_UUID);
            }
            break;

        case HCI_EVENT_DISCONNECTION_COMPLETE: {
            hci_con_handle_t handle = hci_event_disconnection_complete_get_connection_handle(packet);
            if (handle == pu_connection_handle) {
                Serial.println("[PU-BLE] Disconnected from PU. Will retry scan shortly.");
                btstack_run_loop_remove_timer(&pu_refresh_timer);
                pu_connection_handle = HCI_CON_HANDLE_INVALID;
                pu_rx_value_handle = 0;
                pu_send_in_flight = false;
                pu_set_state(PU_STATE_IDLE);
                pu_schedule_reconnect();
            }
            break;
        }

        default:
            break;
    }
}

static void pu_gatt_client_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void) packet_type; (void) channel; (void) size;
    uint8_t event_type = hci_event_packet_get_type(packet);

    switch (event_type) {
        case GATT_EVENT_SERVICE_QUERY_RESULT: {
            gatt_event_service_query_result_get_service(packet, &pu_service);
            pu_service_start_handle = pu_service.start_group_handle;
            Serial.printf("[PU-BLE] NUS service found, handles %u-%u\n",
                          pu_service.start_group_handle, pu_service.end_group_handle);
            break;
        }

        case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT: {
            gatt_client_characteristic_t characteristic;
            gatt_event_characteristic_query_result_get_characteristic(packet, &characteristic);

            bool can_write_no_resp = (characteristic.properties & ATT_PROPERTY_WRITE_WITHOUT_RESPONSE) != 0;
            bool can_write = (characteristic.properties & ATT_PROPERTY_WRITE) != 0;

            Serial.printf("[PU-BLE] char handle=%u properties=0x%02x (write=%d write_no_resp=%d)\n",
                          characteristic.value_handle, characteristic.properties, can_write, can_write_no_resp);

            if ((can_write || can_write_no_resp) && pu_rx_value_handle == 0) {
                pu_rx_value_handle = characteristic.value_handle;
                pu_rx_write_without_response = can_write_no_resp ? true : false;
                Serial.printf("[PU-BLE] selected RX characteristic handle=%u, write_without_response=%d\n",
                              pu_rx_value_handle, pu_rx_write_without_response);
            }
            break;
        }

        case GATT_EVENT_QUERY_COMPLETE: {
            uint8_t status = gatt_event_query_complete_get_att_status(packet);

            if (pu_state == PU_STATE_DISCOVER_SERVICE) {
                if (status != ATT_ERROR_SUCCESS || pu_service_start_handle == 0) {
                    Serial.printf("[PU-BLE] service discovery failed, status=%u. Disconnecting.\n", status);
                    gap_disconnect(pu_connection_handle);
                    break;
                }
                pu_set_state(PU_STATE_DISCOVER_CHAR);
                Serial.println("[PU-BLE] Discovering characteristics...");
                gatt_client_discover_characteristics_for_service(&pu_gatt_client_event_handler, pu_connection_handle, &pu_service);

            } else if (pu_state == PU_STATE_DISCOVER_CHAR) {
                if (pu_rx_value_handle == 0) {
                    Serial.println("[PU-BLE] No writable characteristic found. Disconnecting.");
                    gap_disconnect(pu_connection_handle);
                    break;
                }
                pu_set_state(PU_STATE_READY);
                Serial.println("[PU-BLE] Ready to send commands to PU.");
                // Periodic forced-reconnect DISABLED. It was a workaround for
                // GATT_CLIENT_BUSY writes that has since been properly fixed
                // (pu_write_now() now waits for BTstack's real write-ready
                // signal instead of guessing on a timer -- confirmed 0 errors
                // across 800+ sustained attempts with this disabled). Forcing
                // a disconnect every 20s was adding its own instability
                // instead (each reconnect re-runs the fragile service/
                // characteristic discovery sequence, which has intermittently
                // failed with status=31) and is no longer needed now that the
                // actual bug is fixed. pu_arm_refresh_timer() is left intact
                // below in case this needs to be re-enabled later.
                // pu_arm_refresh_timer();

            } else if (pu_state == PU_STATE_READY) {
                pu_diag_send_attempts++;
                if (status != ATT_ERROR_SUCCESS) {
                    pu_diag_send_errors++;
                    Serial.printf("[PU-BLE] write error (with-response): status=%u\n", status);
                }
            }
            break;
        }

        default:
            break;
    }
}

static void pu_arm_cooldown_timer(void) {
    // Same defensive remove-before-add as pu_schedule_reconnect() above --
    // currently this call site is naturally guarded against double-adds
    // (pu_do_send() won't re-enter until pu_send_in_flight clears), but
    // making every btstack_run_loop_add_timer() call site idempotent costs
    // nothing and rules this class of crash out for good.
    btstack_run_loop_remove_timer(&pu_send_cooldown_timer);
    btstack_run_loop_set_timer_handler(&pu_send_cooldown_timer, &pu_send_cooldown_timer_handler);
    btstack_run_loop_set_timer(&pu_send_cooldown_timer, PU_MIN_SEND_INTERVAL_MS);
    btstack_run_loop_add_timer(&pu_send_cooldown_timer);
}

static void pu_send_cooldown_timer_handler(btstack_timer_source_t *ts) {
    (void) ts;
    pu_send_in_flight = false;
}

// Actually performs the write-without-response send. Only called once
// BTstack confirms (via gatt_client_request_to_write_without_response) that
// a slot is really free -- this may happen synchronously inside pu_do_send()
// if one's free right now, or later once the controller returns a buffer.
// Root cause of the old "stops responding" bug: the previous code fired
// gatt_client_write_value_of_characteristic_without_response() on a fixed
// 250ms polling timer, decoupled from BTstack's actual readiness. When a
// write's underlying ACL buffer hadn't actually cleared yet (e.g. PU or the
// link running behind), that call returned GATT_CLIENT_BUSY (status=148,
// confirmed against btstack's bluetooth.h) -- logged as "write error
// (no-response): status=148". Enough of those in a row and the channel
// backs up until PU appears to freeze entirely. Gating the actual write on
// the real BTstack readiness signal instead of a guessed interval removes
// the busy errors at the source.
static void pu_write_now(void *context) {
    (void) context;

    if (pu_state != PU_STATE_READY || pu_rx_value_handle == 0) {
        pu_send_in_flight = false;
        return;
    }

    uint16_t len = (uint16_t) strlen(pu_send_buf);
    uint8_t status = gatt_client_write_value_of_characteristic_without_response(
        pu_connection_handle, pu_rx_value_handle, len, (uint8_t*) pu_send_buf);
    pu_diag_send_attempts++;
    if (status != ERROR_CODE_SUCCESS) {
        pu_diag_send_errors++;
        Serial.printf("[PU-BLE] write error (no-response): status=%u payload='%s'\n", status, pu_send_buf);
    }

    // Keep the send slot occupied until the cooldown expires.
    // This avoids hammering BTstack when the link is congested and the
    // write-without-response path returns 148 repeatedly.
    pu_arm_cooldown_timer();
}

static void pu_do_send(void *context) {
    (void) context;

    if (pu_state != PU_STATE_READY || pu_rx_value_handle == 0) {
        return;
    }
    if (pu_send_in_flight) {
        return;
    }

    pu_send_in_flight = true;
    pu_send_in_flight_since_ms = pu_now_ms();

    if (pu_rx_write_without_response) {
        pu_can_write_callback_reg.callback = &pu_write_now;
        pu_can_write_callback_reg.context = NULL;
        uint8_t req_status = gatt_client_request_to_write_without_response(&pu_can_write_callback_reg, pu_connection_handle);
        if (req_status != ERROR_CODE_SUCCESS) {
            // Connection likely dropped between the READY check above and
            // here. pu_write_now() will never fire in this case, so clear
            // in_flight immediately rather than waiting on the watchdog.
            pu_diag_send_attempts++;
            pu_diag_send_errors++;
            Serial.printf("[PU-BLE] request_to_write failed: status=%u\n", req_status);
            pu_send_in_flight = false;
        }
        // else: pu_write_now() handles the actual send and clears
        // pu_send_in_flight, whether that happens synchronously right here
        // or asynchronously once BTstack signals a free slot.
    } else {
        uint16_t len = (uint16_t) strlen(pu_send_buf);
        uint8_t status = gatt_client_write_value_of_characteristic(
            &pu_gatt_client_event_handler, pu_connection_handle, pu_rx_value_handle, len, (uint8_t*) pu_send_buf);
        if (status != ERROR_CODE_SUCCESS) {
            pu_diag_send_attempts++;
            pu_diag_send_errors++;
            Serial.printf("[PU-BLE] write error (with-response, queue failed): status=%u payload='%s'\n", status, pu_send_buf);
        }
        // This path's true completion signal is GATT_EVENT_QUERY_COMPLETE
        // in pu_gatt_client_event_handler, which doesn't currently clear
        // pu_send_in_flight -- falls back to the stuck watchdog in
        // pu_send_line. Not the active path (PU's characteristic supports
        // write-without-response, confirmed via logged properties), so left
        // as a fixed-interval fallback rather than rewired right now.
        pu_arm_cooldown_timer();
    }
}

static void pu_maybe_print_diag(void) {
    uint32_t now = pu_now_ms();
    if (now - pu_diag_last_print_ms >= 2000) {
        pu_diag_last_print_ms = now;
        Serial.printf("[PU-BLE] diag: state=%s in_flight=%d attempts=%u errors=%u watchdog_recoveries=%u hci_events=%u adv_reports=%u T=%u S=%u P=%u R=%u last='%s'\n",
                      pu_state_name(pu_state), pu_send_in_flight, pu_diag_send_attempts, pu_diag_send_errors,
                      pu_diag_watchdog_recoveries, pu_diag_total_hci_events, pu_diag_adv_reports,
                      pu_diag_turn_sent, pu_diag_speed_sent, pu_diag_pitch_sent, pu_diag_roll_sent, pu_send_buf);
    }
}

void pu_ble_init(void) {
    pu_hci_event_callback_registration.callback = &pu_packet_handler;
    hci_add_event_handler(&pu_hci_event_callback_registration);

    pu_set_state(PU_STATE_IDLE);
    pu_diag_last_print_ms = pu_now_ms();

    Serial.println("[PU-BLE] Ready (pu_ble_init complete)");

    pu_scan_callback_reg.callback = &pu_do_start_scan;
    pu_scan_callback_reg.context = NULL;
    btstack_run_loop_execute_on_main_thread(&pu_scan_callback_reg);
}

void pu_ble_request_connect(void) {
    if (pu_state != PU_STATE_IDLE) {
        Serial.printf("[PU-BLE] request_connect ignored, state=%s\n", pu_state_name(pu_state));
        return;
    }
    pu_scan_callback_reg.callback = &pu_do_start_scan;
    pu_scan_callback_reg.context = NULL;
    btstack_run_loop_execute_on_main_thread(&pu_scan_callback_reg);
}

bool pu_ble_is_ready(void) {
    return pu_state == PU_STATE_READY;
}

bool pu_send_line(const char* line) {
    pu_maybe_print_diag();

    if (pu_state != PU_STATE_READY || pu_rx_value_handle == 0) {
        return false;
    }

    if (pu_send_in_flight && (pu_now_ms() - pu_send_in_flight_since_ms) > PU_STUCK_WATCHDOG_MS) {
        pu_diag_watchdog_recoveries++;
        Serial.printf("[PU-BLE] WATCHDOG: send stuck in-flight >%ums, force-clearing (recovery #%u)\n",
                      (unsigned) PU_STUCK_WATCHDOG_MS, pu_diag_watchdog_recoveries);
        pu_send_in_flight = false;
    }

    if (pu_send_in_flight) {
        return false;
    }

    size_t len = strlen(line);
    if (len >= sizeof(pu_send_buf) - 2) {
        len = sizeof(pu_send_buf) - 2;
    }
    memcpy(pu_send_buf, line, len);
    if (len == 0 || pu_send_buf[len - 1] != '\n') {
        pu_send_buf[len] = '\n';
        len += 1;
    }
    pu_send_buf[len] = 0;

    pu_send_callback_reg.callback = &pu_do_send;
    pu_send_callback_reg.context = NULL;
    btstack_run_loop_execute_on_main_thread(&pu_send_callback_reg);

    return true;
}

bool pu_send_turn(float value) {
    char line[16];
    snprintf(line, sizeof(line), "T%.2f", (double) value);
    bool ok = pu_send_line(line);
    if (ok) pu_diag_turn_sent++;
    return ok;
}

bool pu_send_speed(float value) {
    char line[16];
    snprintf(line, sizeof(line), "S%.2f", (double) value);
    bool ok = pu_send_line(line);
    if (ok) pu_diag_speed_sent++;
    return ok;
}

bool pu_send_button(int value) {
    char line[16];
    snprintf(line, sizeof(line), "B%d", value);
    return pu_send_line(line);
}

bool pu_send_pitch(float value) {
    char line[16];
    snprintf(line, sizeof(line), "P%.2f", (double) value);
    bool ok = pu_send_line(line);
    if (ok) pu_diag_pitch_sent++;
    return ok;
}

bool pu_send_roll(float value) {
    char line[16];
    snprintf(line, sizeof(line), "R%.2f", (double) value);
    bool ok = pu_send_line(line);
    if (ok) pu_diag_roll_sent++;
    return ok;
}

bool pu_send_text(const char* text) {
    char line[48];
    snprintf(line, sizeof(line), "X%s", text);
    return pu_send_line(line);
}

bool pu_send_drive(float turn, float speed) {
    bool ok;
    if (pu_drive_toggle) {
        ok = pu_send_speed(speed);
    } else {
        ok = pu_send_turn(turn);
    }
    if (ok) pu_drive_toggle = !pu_drive_toggle;
    return ok;
}

// Hysteresis: a HIGHER bar to *enter* driving mode than to *leave* it.
// With a single flat threshold, stick noise/drift hovering right around it
// flips isDriving true/false almost every call, which sends an incoherent
// mix of drive and look-around commands in quick succession -- looks like
// PU "moving but not doing anything logical." Two thresholds with a gap
// between them means the stick has to clearly commit to a state before the
// send mode actually switches, killing that flicker.
#define PU_LOOK_ENTER_DRIVE_THRESHOLD 0.25f  // must exceed this to START driving
#define PU_LOOK_EXIT_DRIVE_THRESHOLD  0.12f  // must drop below this to STOP driving

static bool pu_look_toggle = false;
static bool pu_currently_driving = false;

bool pu_send_drive_or_look(float turn, float speed, float lookPitch, float lookRoll) {
    float mag = fabsf(turn) > fabsf(speed) ? fabsf(turn) : fabsf(speed);

    if (pu_currently_driving) {
        if (mag < PU_LOOK_EXIT_DRIVE_THRESHOLD) {
            pu_currently_driving = false;
        }
    } else {
        if (mag > PU_LOOK_ENTER_DRIVE_THRESHOLD) {
            pu_currently_driving = true;
        }
    }

    if (pu_currently_driving) {
        return pu_send_drive(turn, speed);
    }

    // Stick centered: reuse this same send slot for head-look instead of
    // resending near-zero drive values -- zero extra traffic vs. before.
    bool ok;
    if (pu_look_toggle) {
        ok = pu_send_roll(lookRoll);
    } else {
        ok = pu_send_pitch(lookPitch);
    }
    if (ok) pu_look_toggle = !pu_look_toggle;
    return ok;
}

// Fixed rotation through all four axes -- matches how the original radio
// remote fed Turn/Speed/Pitch/Roll independently rather than picking one
// "mode." PU now applies each message immediately via runKeyValueCommand
// (same as radio), so there's nothing to decide here: always send the next
// axis in the rotation regardless of its value. If the underlying send is
// skipped (still in cooldown / not ready), the rotation index does NOT
// advance, so the same axis is retried next tick instead of silently
// skipping it.
static int pu_axis_rotation = 0;

bool pu_send_all_axes(float turn, float speed, float pitch, float roll) {
    bool ok;
    switch (pu_axis_rotation) {
        case 0:  ok = pu_send_turn(turn);   break;
        case 1:  ok = pu_send_speed(speed); break;
        case 2:  ok = pu_send_pitch(pitch); break;
        default: ok = pu_send_roll(roll);   break;
    }
    if (ok) {
        pu_axis_rotation = (pu_axis_rotation + 1) % 4;
    }
    return ok;
}

// Combined send: all four axes in one UART line ("A<turn>,<speed>,<pitch>,
// <roll>\n") instead of the 4-way rotation above, so a single
// PU_MIN_SEND_INTERVAL_MS cooldown covers a full update of every axis
// instead of gating each axis separately. Worst case with the rotation was
// ~4x PU_MIN_SEND_INTERVAL_MS before any one axis (e.g. Pitch) saw fresh
// data; this cuts that to 1x. Buffer sizing: worst case four values like
// "-0.99" (5 chars) plus the 'A' prefix and 3 commas is ~24 chars, well
// under pu_send_buf's 64-byte cap enforced in pu_send_line().
bool pu_send_all_axes_combined(float turn, float speed, float pitch, float roll) {
    char line[48];
    snprintf(line, sizeof(line), "A%.2f,%.2f,%.2f,%.2f",
             (double) turn, (double) speed, (double) pitch, (double) roll);
    bool ok = pu_send_line(line);
    if (ok) {
        pu_diag_turn_sent++;
        pu_diag_speed_sent++;
        pu_diag_pitch_sent++;
        pu_diag_roll_sent++;
    }
    return ok;
}
