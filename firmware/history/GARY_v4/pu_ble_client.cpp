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

static btstack_timer_source_t pu_send_cooldown_timer;
static btstack_timer_source_t pu_reconnect_timer;

static char pu_send_buf[64];
static bool pu_send_in_flight = false;
static uint32_t pu_send_in_flight_since_ms = 0;
static bool pu_drive_toggle = false;

static uint32_t pu_diag_send_attempts = 0;
static uint32_t pu_diag_send_errors = 0;
static uint32_t pu_diag_total_hci_events = 0;
static uint32_t pu_diag_adv_reports = 0;
static uint32_t pu_diag_watchdog_recoveries = 0;
static uint32_t pu_diag_last_print_ms = 0;

static void pu_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void pu_gatt_client_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void pu_send_cooldown_timer_handler(btstack_timer_source_t *ts);
static void pu_reconnect_timer_handler(btstack_timer_source_t *ts);
static void pu_do_start_scan(void *context);
static void pu_do_connect(void *context);
static void pu_do_send(void *context);
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
    btstack_run_loop_set_timer_handler(&pu_reconnect_timer, &pu_reconnect_timer_handler);
    btstack_run_loop_set_timer(&pu_reconnect_timer, PU_RECONNECT_DELAY_MS);
    btstack_run_loop_add_timer(&pu_reconnect_timer);
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
    btstack_run_loop_set_timer_handler(&pu_send_cooldown_timer, &pu_send_cooldown_timer_handler);
    btstack_run_loop_set_timer(&pu_send_cooldown_timer, PU_MIN_SEND_INTERVAL_MS);
    btstack_run_loop_add_timer(&pu_send_cooldown_timer);
}

static void pu_send_cooldown_timer_handler(btstack_timer_source_t *ts) {
    (void) ts;
    pu_send_in_flight = false;
}

static void pu_do_send(void *context) {
    (void) context;

    if (pu_state != PU_STATE_READY || pu_rx_value_handle == 0) {
        return;
    }
    if (pu_send_in_flight) {
        return;
    }

    uint16_t len = (uint16_t) strlen(pu_send_buf);
    uint8_t status;

    pu_send_in_flight = true;
    pu_send_in_flight_since_ms = pu_now_ms();

    if (pu_rx_write_without_response) {
        status = gatt_client_write_value_of_characteristic_without_response(
            pu_connection_handle, pu_rx_value_handle, len, (uint8_t*) pu_send_buf);
        pu_diag_send_attempts++;
        if (status != ERROR_CODE_SUCCESS) {
            pu_diag_send_errors++;
            Serial.printf("[PU-BLE] write error (no-response): status=%u payload='%s'\n", status, pu_send_buf);
        }
    } else {
        status = gatt_client_write_value_of_characteristic(
            &pu_gatt_client_event_handler, pu_connection_handle, pu_rx_value_handle, len, (uint8_t*) pu_send_buf);
        if (status != ERROR_CODE_SUCCESS) {
            pu_diag_send_attempts++;
            pu_diag_send_errors++;
            Serial.printf("[PU-BLE] write error (with-response, queue failed): status=%u payload='%s'\n", status, pu_send_buf);
        }
    }

    pu_arm_cooldown_timer();
}

static void pu_maybe_print_diag(void) {
    uint32_t now = pu_now_ms();
    if (now - pu_diag_last_print_ms >= 2000) {
        pu_diag_last_print_ms = now;
        Serial.printf("[PU-BLE] diag: state=%s in_flight=%d attempts=%u errors=%u watchdog_recoveries=%u hci_events=%u adv_reports=%u last='%s'\n",
                      pu_state_name(pu_state), pu_send_in_flight, pu_diag_send_attempts, pu_diag_send_errors,
                      pu_diag_watchdog_recoveries, pu_diag_total_hci_events, pu_diag_adv_reports, pu_send_buf);
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
    return pu_send_line(line);
}

bool pu_send_speed(float value) {
    char line[16];
    snprintf(line, sizeof(line), "S%.2f", (double) value);
    return pu_send_line(line);
}

bool pu_send_button(int value) {
    char line[16];
    snprintf(line, sizeof(line), "B%d", value);
    return pu_send_line(line);
}

bool pu_send_pitch(float value) {
    char line[16];
    snprintf(line, sizeof(line), "P%.2f", (double) value);
    return pu_send_line(line);
}

bool pu_send_roll(float value) {
    char line[16];
    snprintf(line, sizeof(line), "R%.2f", (double) value);
    return pu_send_line(line);
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

#define PU_LOOK_DEADZONE 0.15f

static bool pu_look_toggle = false;

bool pu_send_drive_or_look(float turn, float speed, float lookPitch, float lookRoll) {
    bool isDriving = (fabsf(turn) > PU_LOOK_DEADZONE) || (fabsf(speed) > PU_LOOK_DEADZONE);

    if (isDriving) {
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
