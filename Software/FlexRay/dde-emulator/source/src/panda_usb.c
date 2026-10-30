#include "panda_usb.h"
#include "pico_fr_record.h"
#if PICO_FLEXRAY_DDE_EMU
#include "dde_emu.h"
#endif
#if PICO_FLEXRAY_BOARD_V2
#include "v2_cantest.h"
#include "board_pico_flexray_v2.h"
#include "pico_can_record.h"
#endif
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "pico/bootrom.h"
#include "hardware/watchdog.h"
#include "tusb.h"
#include "flexray_blocker.h"
#include "flexray_frame.h"
#include "flexray_fifo.h"
#include "flexray_injector.h"
#include <string.h>

// Add near top after includes
static absolute_time_t last_usb_activity = 0;

// Bitmask-to-decimal: each set bit contributes its channel number as a digit.
// Index is the 4-bit source bitmask (FR1=bit3, FR2=bit2, FR3=bit1, FR4=bit0).
static const uint8_t source_decimal[16] = {
    0,    // 0b0000  unknown
    4,    // 0b0001  FR4
    3,    // 0b0010  FR3
    34,   // 0b0011  FR3+FR4
    2,    // 0b0100  FR2
    24,   // 0b0101  FR2+FR4
    23,   // 0b0110  FR2+FR3
    234,  // 0b0111  FR2+FR3+FR4
    1,    // 0b1000  FR1
    14,   // 0b1001  FR1+FR4
    13,   // 0b1010  FR1+FR3
    134,  // 0b1011  FR1+FR3+FR4
    12,   // 0b1100  FR1+FR2
    124,  // 0b1101  FR1+FR2+FR4
    123,  // 0b1110  FR1+FR2+FR3
    0,    // 0b1111  all (overflow, should not happen)
};

// FlexRay FIFO
static flexray_fifo_t flexray_fifo;

// TS-CAPTURE: FlexRay record format on the bulk-IN stream (pico_fr_record.h).
// 1 = stock (default, and again after every USB unmount), 2 = SavvyCAN device-us,
// 3 = high-resolution timestamps. Set with vendor request 0xa0.
static uint8_t record_format = PICO_FR_FORMAT_STOCK;
extern void capture_fill_ts_info(pico_ts_info_t *ti); // main.c

#if PICO_FLEXRAY_BOARD_V2
// CAN records (V2 board MCP251863) waiting for the bulk-IN endpoint. Core0
// only: filled from the main loop, drained by try_send_from_fifo() (main loop
// and tud_task callbacks). Never blocks: a full queue drops and counts.
#define CAN_REC_QUEUE_LEN 256
static pico_can_record_t can_q[CAN_REC_QUEUE_LEN];
static uint16_t can_q_w, can_q_r;
static bool can_records_enabled = V2_CAN_RECORDS_DEFAULT_ON;
static panda_can_q_stats_t can_q_stats;
static inline bool can_q_empty(void) { return can_q_w == can_q_r; }
static void can_q_clear(void) { can_q_w = can_q_r = 0; }
#endif

// For delayed reset/bootloader
static bool pending_reset = false;
static bool pending_bootloader = false;

// Global state
static struct
{
    uint8_t hw_type;
    uint8_t safety_model;
    bool initialized;
    uint16_t alternative_experience;
} panda_state;

// Internal functions
static bool handle_control_read(uint8_t rhport, tusb_control_request_t const *request);
static bool handle_control_write(uint8_t rhport, tusb_control_request_t const *request);
static bool handle_control_data_stage(tusb_control_request_t const *request, uint8_t const *data, uint16_t len);
static bool try_send_from_fifo(const char *context);
// ------------------------------------------------------------
// Vendor OUT protocol (host -> device)
//  op 0x90: Push override replacement slice
//    [0x90][u16 id][u8 base][u16 len][len bytes slice]
//    - id must match a trigger_rule_t.target_id; base must match rule.cycle_base
//    - len must equal rule.replace_len
//  op 0x91: Set injector enable
//    [0x91][u8 enabled]
//  op 0x92: Replace forwarding filter table
//    [0x92][u8 enabled][u8 count][count * ([u16 id][u8 direction_mask])]
//    - direction_mask uses FLEXRAY_FILTER_DIR_* bits from flexray_blocker.h
//  op 0x93: Clear forwarding filter table
//  op 0x95: Set whitelist-default directions for forwarding filter
//    [0x95][u8 direction_mask]; listed directions block by default, rule matches pass
// ------------------------------------------------------------
static void handle_vendor_out_payload(const uint8_t *data, uint16_t len)
{
    uint32_t off = 0;
    while ((uint16_t)(len - off) >= 1) {
        uint8_t op = data[off++];
        if (op == 0x90) {
            if ((uint16_t)(len - off) < 5) {
                break;
            }
            uint16_t id = (uint16_t)(data[off] | ((uint16_t)data[off + 1] << 8));
            uint8_t base = data[off + 2];
            uint16_t flen = (uint16_t)(data[off + 3] | ((uint16_t)data[off + 4] << 8));
            off += 5;
            if ((uint16_t)(len - off) < flen) {
                break;
            }
            (void)injector_submit_override(id, base, flen, &data[off]);
            off += flen;
        } else if (op == 0x91) {
            if ((uint16_t)(len - off) < 1) {
                break;
            }
            bool en = data[off++] != 0;
            injector_set_enabled(en);
        } else if (op == 0x92) {
            if ((uint16_t)(len - off) < 2) {
                break;
            }
            bool enabled = data[off++] != 0;
            uint8_t count = data[off++];
            if (count > FLEXRAY_FILTER_MAX_RULES ||
                (uint16_t)(len - off) < (uint16_t)(count * 3u)) {
                break;
            }

            uint16_t ids[FLEXRAY_FILTER_MAX_RULES];
            uint8_t masks[FLEXRAY_FILTER_MAX_RULES];
            for (uint8_t i = 0; i < count; i++) {
                ids[i] = (uint16_t)(data[off] | ((uint16_t)data[off + 1] << 8));
                masks[i] = data[off + 2];
                off += 3;
            }
            if (flexray_filter_set(count, ids, masks)) {
                flexray_filter_set_enabled(enabled);
            }
        } else if (op == 0x93) {
            flexray_filter_clear();
            flexray_filter_set_enabled(false);
        } else if (op == 0x95) {
            if ((uint16_t)(len - off) < 1) {
                break;
            }
            (void)flexray_filter_set_whitelist_defaults(data[off++]);
        } else if (op == 0x00) {
            continue;
        } else {
            // Unknown op: stop parsing this buffer
            break;
        }
    }
}


// Placeholder for git version
const char *GITLESS_REVISION = "dev";

void panda_usb_init(void)
{
    // Initialize TinyUSB
    tud_init(0);

    // Initialize FlexRay FIFO
    flexray_fifo_init(&flexray_fifo);

    // Initialize panda state
    panda_state.hw_type = HW_TYPE_RED_PANDA;
    panda_state.safety_model = SAFETY_SILENT;
    panda_state.initialized = true;
    panda_state.alternative_experience = 0;

    printf("Panda USB initialized - VID:0x%04x PID:0x%04x\n", 0x3801, 0xddcc);
    last_usb_activity = get_absolute_time();
}

void panda_usb_task(void)
{
    tud_task();
}

// TinyUSB vendor control transfer callback - this overrides the weak default implementation
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    static tusb_control_request_t current_request_local;
    static uint8_t control_buffer[CFG_TUD_ENDPOINT0_SIZE];

    switch (stage)
    {
    case CONTROL_STAGE_SETUP:
        // Store the request. We need it in the DATA stage to dispatch correctly.
        current_request_local = *request;
        pending_reset = false;
        pending_bootloader = false;

        if (request->bmRequestType & TUSB_DIR_IN_MASK)
        {
            // IN request
            return handle_control_read(rhport, request);
        }
        else
        {
            // OUT request
            if (request->wLength > 0)
            {
                // OUT with data: provide buffer for TinyUSB to receive data
                return tud_control_xfer(rhport, request, control_buffer, request->wLength);
            }
            else
            {
                // OUT with no data
                return handle_control_write(rhport, request);
            }
        }

    case CONTROL_STAGE_DATA:
        // Handle OUT data stage
        if (!(current_request_local.bmRequestType & TUSB_DIR_IN_MASK) && current_request_local.wLength > 0)
        {
            handle_control_data_stage(&current_request_local, control_buffer, current_request_local.wLength);
            // tud_control_status(rhport, request);
        }
        return true;

    case CONTROL_STAGE_ACK:
        // Execute delayed operations after status stage
        if (pending_reset)
        {
            watchdog_reboot(0, 0, 0);
        }
        else if (pending_bootloader)
        {
            reset_usb_boot(0, 0);
        }
        return true;

    default:
        // Let TinyUSB handle other stages like DATA
        return true;
    }
}

static bool handle_control_read(uint8_t rhport, tusb_control_request_t const *request)
{
    uint8_t response_data[64] = {0};
    uint16_t response_len = 0;

    switch (request->bRequest)
    {
    case PANDA_GET_HW_TYPE:
        // Return hardware type (Red Panda = 4 for CAN-FD support)
        response_data[0] = panda_state.hw_type;
        response_len = 1;
        // printf("Control Read: GET_HW_TYPE -> %d\n", response_data[0]);
        break;

    case PANDA_GET_MICROSECOND_TIMER:
        // Return microsecond timer value
        {
            uint32_t timer_val = time_us_32();
            memcpy(response_data, &timer_val, sizeof(timer_val));
            response_len = sizeof(timer_val);
            // printf("Control Read: GET_MICROSECOND_TIMER -> %lu\n", timer_val);
        }
        break;

#if PICO_FLEXRAY_DDE_EMU
    case PICOFLEX_GET_EMU:
        {
            // 192 bytes (v3) do not fit response_data[64] and a multi-packet control IN
            // reads its buffer after this function returns: use a static buffer. TinyUSB
            // sends min(wLength, 192), so a v1 host asking for 64 gets the v1 layout and
            // a v2 host asking for 128 the v2 layout.
            static dde_emu_stats_t es;
            dde_emu_fill_stats(&es);
            return tud_control_xfer(rhport, request, &es, sizeof(es));
        }
#endif

    case PICOFLEX_GET_TS_INFO:
        {
            pico_ts_info_t ti;
            capture_fill_ts_info(&ti);
            ti.record_format = record_format;
            memcpy(response_data, &ti, sizeof(ti));
            response_len = sizeof(ti);
        }
        break;

    case PANDA_GET_FAN_RPM:
        // TODO: Implement fan RPM reading
        {
            uint16_t fan_rpm = 0; // Placeholder
            memcpy(response_data, &fan_rpm, sizeof(fan_rpm));
            response_len = sizeof(fan_rpm);
            // printf("Control Read: GET_FAN_RPM -> %d\n", fan_rpm);
        }
        break;

    case PANDA_GET_CAN_HEALTH_STATS:
        struct can_health_t * can_health = (struct can_health_t*)response_data;
        can_health->can_speed = 0;
        can_health->can_data_speed = 0;
        can_health->canfd_enabled = 0;
        can_health->brs_enabled = 0;
        can_health->canfd_non_iso = 0;
        can_health->total_rx_cnt = 0;
        can_health->total_rx_lost_cnt = 0;
#if PICO_FLEXRAY_BOARD_V2
        v2_can_fill_health(can_health);
#endif
        response_len = sizeof(struct can_health_t);
        memcpy(response_data, can_health, response_len);
        // printf("Control Read: GET_CAN_HEALTH_STATS\n");
        break;

    // case PANDA_GET_MCU_UID:
    // {
    //     pico_unique_board_id_t uid;
    //     pico_get_unique_board_id(&uid);
    //     // On Pico, UID is 8 bytes. Panda expects 12. Pad with 0s.
    //     memset(response_data, 0, 12);
    //     memcpy(response_data, uid.id, PICO_UNIQUE_BOARD_ID_SIZE_BYTES);
    //     response_len = 12;
    //     // printf("Control Read: GET_MCU_UID\n");
    // }
    // break;

    case PANDA_GET_HEALTH_PACKET:
        struct health_t * health = (struct health_t*)response_data;
        health->uptime_pkt = 0;
        health->voltage_pkt = 0;
        health->current_pkt = 0;
        health->safety_tx_blocked_pkt = 0;
        health->safety_rx_invalid_pkt = 0;
        health->tx_buffer_overflow_pkt = 0;
        health->rx_buffer_overflow_pkt = 0;
        health->faults_pkt = 0;
        health->ignition_line_pkt = 1;
        health->ignition_can_pkt = 1;
        health->controls_allowed_pkt = 1;
        health->car_harness_status_pkt = 1;
        health->safety_mode_pkt = 17;
        health->safety_param_pkt = 0;
        health->fault_status_pkt = 0;
        health->power_save_enabled_pkt = 0;
        health->heartbeat_lost_pkt = 0;
        health->alternative_experience_pkt = panda_state.alternative_experience;
        health->interrupt_load_pkt = 0;
        health->fan_power = 0;
        health->safety_rx_checks_invalid_pkt = 0;
        health->spi_error_count_pkt = 0;
        health->sbu1_voltage_mV = 0;
        health->sbu2_voltage_mV = 0;
        health->som_reset_triggered = 0;
        response_len = sizeof(struct health_t);
        memcpy(response_data, health, response_len);
        // printf("Control Read: GET_HEALTH_PACKET\n");
        break;

    case PANDA_GET_SIGNATURE_PART1:
        response_len = 64;
        memset(response_data, 0, response_len);
        // printf("Control Read: PANDA_GET_SIGNATURE_PART1\n");
        break;

    case PANDA_GET_SIGNATURE_PART2:
        response_len = 64;
        memset(response_data, 0, response_len);
        // printf("Control Read: PANDA_GET_SIGNATURE_PART2\n");
        break;

    case PANDA_GET_GIT_VERSION:
        response_len = strlen(GITLESS_REVISION);
        memcpy(response_data, GITLESS_REVISION, response_len);
        // printf("Control Read: PANDA_GET_GIT_VERSION\n");
        break;


    case PANDA_GET_VERSIONS:
        response_data[0] = 17;
        response_data[1] = 4;
        response_data[2] = 5;
        response_len = 3;
        // printf("Control Read: PANDA_GET_VERSIONS\n");
        break;

    case PANDA_UART_READ:
        response_len = 0;
        memset(response_data, 'c', response_len);
        // printf("Control Read: PANDA_UART_READ\n");
        break;

    default:
        printf("Control Read: Unknown request 0x%02x\n", request->bRequest);
        return false;
    }

    // if (response_len >= 0)
    // {
        return tud_control_xfer(rhport, request, response_data, response_len);
    // }

    // return false;
}

static bool handle_control_write(uint8_t rhport, tusb_control_request_t const *request)
{
    bool handled = false;

    // Handle write requests without data - just process them
    switch (request->bRequest)
    {
    case PANDA_RESET_CAN_COMMS:
        // printf("Control Write: RESET_CAN_COMMS (request=0x%02x)\n", request->bRequest);
        flexray_fifo_init(&flexray_fifo);
#if PICO_FLEXRAY_BOARD_V2
        can_q_clear();
#endif
        handled = true;
        break;

    case PICOFLEX_SET_RECORD_FORMAT:
        // TS-CAPTURE: 1 stock, 2 SavvyCAN device-us, 3 high-resolution. Anything else stalls.
        if (request->wValue < PICO_FR_FORMAT_STOCK || request->wValue > PICO_FR_FORMAT_HIRES) {
            return false;
        }
        if (record_format != (uint8_t)request->wValue) {
            // drop frames queued in the old format's context (encoding happens at send time)
            flexray_fifo_init(&flexray_fifo);
        }
        record_format = (uint8_t)request->wValue;
        printf("Control Write: SET_RECORD_FORMAT -> %u\n", (unsigned)record_format);
        handled = true;
        break;

#if PICO_FLEXRAY_DDE_EMU
    case PICOFLEX_SET_EMU:
        // DDE emulator TX: 1 = enable / keepalive, 0 = disable and clear a latched fault.
        if (request->wValue > 1u) {
            return false;
        }
        dde_emu_host_set(request->wValue);
        handled = true;
        break;
#endif

#if PICO_FLEXRAY_BOARD_V2
    case PICOFLEX_SET_CAN_RECORDS:
        // Host opts in to CAN records in the bulk-IN stream (see pico_can_record.h).
        can_records_enabled = (request->wValue & 1) != 0;
        if (!can_records_enabled) can_q_clear();
        printf("Control Write: SET_CAN_RECORDS -> %s\n", can_records_enabled ? "on" : "off");
        handled = true;
        break;
#endif

    case PANDA_SET_CAN_FD_AUTO_SWITCH:
        // printf("Control Write: SET_CAN_FD_AUTO_SWITCH -> %d\n", request->wValue);
        handled = true;
        break;

    case PANDA_SET_OBD_CAN_MUX_MODE:
        handled = true;
        break;

    case PANDA_SET_SAFETY_MODEL:
        panda_state.safety_model = request->wValue;
        // printf("Control Write: SET_SAFETY_MODEL -> %d, param %d\n", request->wValue, request->wIndex);
        handled = true;
        break;

    case PANDA_SET_ALT_EXPERIENCE:
        // Host writes two 16-bit values via wValue/wIndex. Save the first as alternative_experience.
        panda_state.alternative_experience = (uint16_t)request->wValue;
        handled = true;
        break;

    case PANDA_SET_CAN_SPEED_KBPS:
    case PANDA_SET_CAN_FD_DATA_BITRATE:
        // printf("Control Write: SET_CAN_SPEED_KBPS -> %d\n", request->wValue);
        handled = true;
        break;

    case PANDA_HEARTBEAT:
        // printf("Control Write: HEARTBEAT\n");
        handled = true;
        break;

    case PANDA_SET_IR_POWER:
        // printf("Control Write: SET_IR_POWER -> %d\n", request->wValue);
        handled = true;
        break;

    case PANDA_SET_FAN_POWER:
        // printf("Control Write: SET_FAN_POWER -> %d\n", request->wValue);
        handled = true;
        break;

    case PANDA_ENTER_BOOTLOADER_MODE:
        // printf("Control Write: ENTER_BOOTLOADER_MODE, mode=%d\n", request->wValue);
        if (request->wValue == 0)
        { // Bootloader
            pending_bootloader = true;
        }
        handled = true;
        break;

    case PANDA_SYSTEM_RESET:
        // printf("Control Write: SYSTEM_RESET\n");
        pending_reset = true;
        handled = true;
        break;

    case PANDA_SET_POWER_SAVE_STATE:
        // printf("Control Write: SET_POWER_SAVE_STATE -> %d\n", request->wValue);
        handled = true;
        break;

    case PANDA_DISABLE_HEARTBEAT_CHECKS:
        // printf("Control Write: DISABLE_HEARTBEAT_CHECKS\n");
        handled = true;
        break;

    default:
        printf("Control Write: Unknown request 0x%02x\n", request->bRequest);
        return false;
    }

    if (handled)
    {
        return tud_control_status(rhport, request);
    }

    return false;
}

static bool handle_control_data_stage(tusb_control_request_t const *request, uint8_t const *data, uint16_t len)
{
    // Process the received data for different commands
    switch (request->bRequest)
    {
    case PANDA_SET_CAN_FD_AUTO_SWITCH:
        printf("Control Data: SET_CAN_FD_AUTO_SWITCH -> %d\n", request->wValue);
        return true;

    case PANDA_SET_CAN_SPEED_KBPS:
        if (len >= 4) // Expect at least bus_id (2 bytes) + speed (2 bytes)
        {
            uint16_t bus_id = data[0] | (data[1] << 8);
            uint16_t speed_kbps = data[2] | (data[3] << 8);

            if (bus_id < 3) // We support up to 3 CAN buses
            {
                printf("Control Data: SET_CAN_SPEED_KBPS bus=%d speed=%d kbps\n", bus_id, speed_kbps);
            }
            else
            {
                printf("Control Data: SET_CAN_SPEED_KBPS invalid bus_id=%d\n", bus_id);
            }
        }
        else
        {
            printf("Control Data: SET_CAN_SPEED_KBPS insufficient data (got %d bytes)\n", len);
        }
        return true;

    case PANDA_SET_CAN_FD_DATA_BITRATE: // Renamed from SET_DATA_SPEED_KBPS
        if (len >= 4)                   // Expect at least bus_id (2 bytes) + speed (2 bytes)
        {
            uint16_t bus_id = data[0] | (data[1] << 8);
            uint16_t data_speed_kbps = data[2] | (data[3] << 8);

            if (bus_id < 3) // We support up to 3 CAN buses
            {
                printf("Control Data: SET_CAN_FD_DATA_BITRATE bus=%d data_speed=%d kbps\n", bus_id, data_speed_kbps);
            }
            else
            {
                printf("Control Data: SET_CAN_FD_DATA_BITRATE invalid bus_id=%d\n", bus_id);
            }
        }
        else
        {
            printf("Control Data: SET_CAN_FD_DATA_BITRATE insufficient data (got %d bytes)\n", len);
        }
        return true;

    default:
        printf("Control Data: Unexpected request 0x%02x with %d bytes\n", request->bRequest, len);
        return false;
    }
}

//--------------------------------------------------------------------+
// TinyUSB Device State Callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
    printf("USB Device mounted\n");
    last_usb_activity = get_absolute_time();
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
    printf("USB Device unmounted - clearing application state\n");
    record_format = PICO_FR_FORMAT_STOCK; // a new host gets stock records until it asks
#if PICO_FLEXRAY_BOARD_V2
    can_records_enabled = V2_CAN_RECORDS_DEFAULT_ON; // a new host has to opt in again
    can_q_clear();
#endif

    // Reset control transfer state - not strictly needed with new design but good practice
    // Reset application state but keep device configuration
    // Don't reset panda_state entirely as it may contain valid configuration

    printf("USB unmount completed - ready for reconnection\n");
}

// Invoked when usb bus is suspended
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
    printf("USB Device suspended\n");
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
    printf("USB Device resumed\n");
}

//--------------------------------------------------------------------+
// TinyUSB Vendor Class Callbacks
//--------------------------------------------------------------------+

// Invoked when received data from host via OUT endpoint
void tud_vendor_rx_cb(uint8_t itf, uint8_t const *buffer, uint16_t bufsize)
{
    (void)itf;
    uint8_t rxbuf[512];
    uint16_t total = 0;

    if (bufsize > sizeof(rxbuf)) {
        bufsize = sizeof(rxbuf);
    }
    if (bufsize > 0) {
        memcpy(rxbuf, buffer, bufsize);
        total = bufsize;
    }

    // Drain additional bytes before parsing so larger filter tables stay whole.
    while (tud_vendor_available()) {
        uint32_t space = (uint32_t)(sizeof(rxbuf) - total);
        if (space == 0) break;
        uint32_t n = tud_vendor_read(rxbuf + total, space);
        if (n == 0) break;
        total = (uint16_t)(total + n);
    }

    if (total > 0) {
        handle_vendor_out_payload(rxbuf, total);
    }
    last_usb_activity = get_absolute_time();
}

// Invoked when a transfer on Bulk IN endpoint is complete
void tud_vendor_tx_cb(uint8_t itf, uint32_t sent_bytes)
{
    (void)itf;
    (void)sent_bytes;

    // After a transfer is complete, try to send the next batch of data
    try_send_from_fifo("tx_cb trigger");
}

#if PICO_FLEXRAY_BOARD_V2
bool panda_can_record_push(const pico_can_record_t *r)
{
    if (!can_records_enabled || !tud_vendor_mounted()) {
        can_q_stats.not_streamed++;
        return false;
    }
    uint16_t next = (uint16_t)((can_q_w + 1) % CAN_REC_QUEUE_LEN);
    if (next == can_q_r) {
        can_q_stats.dropped++;
        try_send_from_fifo("can_push_full");
        return false;
    }
    can_q[can_q_w] = *r;
    can_q_w = next;
    can_q_stats.queued++;
    try_send_from_fifo("can_push");
    return true;
}

void panda_can_get_q_stats(panda_can_q_stats_t *st)
{
    *st = can_q_stats;
    st->enabled = can_records_enabled;
    st->depth = (uint16_t)((can_q_w + CAN_REC_QUEUE_LEN - can_q_r) % CAN_REC_QUEUE_LEN);
}
#endif

bool panda_flexray_fifo_push(const flexray_frame_t *frame)
{
    try_send_from_fifo("fifo_push");
    return flexray_fifo_push(&flexray_fifo, frame);
}

// Centralized function to trigger USB transmission from FIFO
static bool try_send_from_fifo(const char *context)
{
    (void)context;
#if PICO_FLEXRAY_BOARD_V2
    bool can_pending = !can_q_empty();
#else
    const bool can_pending = false;
#endif
    if (!tud_vendor_mounted() || (flexray_fifo_is_empty(&flexray_fifo) && !can_pending))
    {
        return false;
    }

    // Minimum record: 2-byte length + 1-byte source + 5-byte header + 0 payload + 3-byte CRC = 11 bytes
    const uint32_t MIN_RECORD_SIZE = 11u;

    uint32_t available_space = tud_vendor_write_available();
    if (available_space < MIN_RECORD_SIZE)
    {
        return false;
    }

    uint32_t total_sent = 0;
    bool sent_something = false;

#if PICO_FLEXRAY_BOARD_V2
    // CAN records first: small (14..22 B) and bounded per call, so FlexRay
    // records still go out in the same call. Whole records only.
    for (int n = 0; n < 32 && !can_q_empty(); n++)
    {
        const pico_can_record_t *r = &can_q[can_q_r];
        uint8_t cbuf[PICO_CAN_REC_MAX_WIRE];
        size_t cw = pico_can_record_encode(r, cbuf);
        if (available_space < cw) break;
        uint32_t written = tud_vendor_write(cbuf, (uint32_t)cw);
        if (written != cw) break;
        can_q_r = (uint16_t)((can_q_r + 1) % CAN_REC_QUEUE_LEN);
        can_q_stats.sent++;
        sent_something = true;
        total_sent += written;
        available_space = tud_vendor_write_available();
    }
#endif

    while (!flexray_fifo_is_empty(&flexray_fifo))
    {
        flexray_frame_t frame;
        // Peek first to preserve order in case we cannot send now
        if (!flexray_fifo_peek(&flexray_fifo, &frame))
        {
            break;
        }

        uint16_t payload_len_bytes = (uint16_t)(frame.payload_length_words * 2u);

        // Reconstruct 5-byte header
        pico_fr_record_t rec;
        rec.source = source_decimal[frame.source & 0x0F];
        rec.flags = frame.ts_flags;
        rec.header[0] = (uint8_t)((frame.indicators << 3) | ((frame.frame_id >> 8) & 0x07));
        rec.header[1] = (uint8_t)(frame.frame_id & 0xFF);
        rec.header[2] = (uint8_t)((frame.payload_length_words << 1) | ((frame.header_crc >> 10) & 0x01));
        rec.header[3] = (uint8_t)((frame.header_crc >> 2) & 0xFF);
        rec.header[4] = (uint8_t)(((frame.header_crc & 0x03) << 6) | (frame.cycle_count & 0x3F));
        rec.payload = frame.payload;
        rec.payload_len = payload_len_bytes;
        rec.crc24 = frame.frame_crc;
        rec.t_us = frame.ts_us;
        rec.t_fine = frame.ts_fine;
        rec.seq = frame.seq;

        // TS-CAPTURE: encode in the host-selected format (1 stock / 2 us / 3 high-res)
        uint8_t outbuf[PICO_FR_MAX_WIRE];
        size_t w = pico_fr_record_encode(record_format, &rec, outbuf);
        if (available_space < w)
        {
            // Not enough space for the head frame; stop and retry later
            break;
        }

        // Write
        uint32_t written = tud_vendor_write(outbuf, (uint32_t)w);
        if (written != w)
        {
            // On partial write, stop loop; data will be retried next call
            break;
        }
        // Now we can safely pop the frame since it has been fully queued to USB
        (void)flexray_fifo_pop(&flexray_fifo, &frame);
        sent_something = true;
        total_sent += written;
        available_space = tud_vendor_write_available();

        // If buffer space drops low, flush early to free FIFO in USB core
        if (available_space < MIN_RECORD_SIZE)
        {
            break;
        }
    }

    if (sent_something)
    {
        tud_vendor_write_flush();
        return true;
    }
    return false;
}
