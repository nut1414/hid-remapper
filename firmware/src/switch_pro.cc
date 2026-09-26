#include <cstring>

#include <tusb.h>

#include "platform.h"
#include "switch_pro.h"

// Report 0x30 is 63 bytes after its ID: two status bytes, three button
// bytes, four packed 12-bit stick axes, vibrator status, 3 IMU samples,
// and 15 reserved bytes. Reports 0x21/0x81 carry protocol replies.
const uint8_t switch_pro_report_descriptor[] = {
    0x05, 0x01, 0x09, 0x05, 0xA1, 0x01,       // Game Pad application
    0x85, 0x30,                               // Standard full input report
    0x06, 0x00, 0xFF, 0x09, 0x01,
    0x15, 0x00, 0x26, 0xFF, 0x00,
    0x75, 0x08, 0x95, 0x02, 0x81, 0x03,       // Timer, battery/connection
    0x05, 0x09, 0x19, 0x01, 0x29, 0x18,
    0x25, 0x01, 0x75, 0x01, 0x95, 0x18,
    0x81, 0x02,                               // 24 button bits
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31,
    0x09, 0x32, 0x09, 0x35,
    0x26, 0xFF, 0x0F, 0x75, 0x0C, 0x95, 0x04,
    0x81, 0x02,                               // Packed sticks
    0x06, 0x00, 0xFF, 0x09, 0x02,
    0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x34,
    0x81, 0x02,                               // Vibrator, IMU, reserved
    0x85, 0x21, 0x09, 0x03, 0x95, 0x3F, 0x81, 0x02,
    0x85, 0x81, 0x09, 0x04, 0x95, 0x3F, 0x81, 0x02,
    0x85, 0x01, 0x09, 0x05, 0x95, 0x3F, 0x91, 0x02,
    0x85, 0x10, 0x09, 0x06, 0x95, 0x3F, 0x91, 0x02,
    0x85, 0x80, 0x09, 0x07, 0x95, 0x3F, 0x91, 0x02,
    0x85, 0x82, 0x09, 0x08, 0x95, 0x3F, 0x91, 0x02,
    0xC0,
};

static_assert(sizeof(switch_pro_report_descriptor) == switch_pro_report_descriptor_length,
              "Update the Switch Pro HID descriptor length");

namespace {

constexpr uint8_t REPORT_LEN = 63;
constexpr uint8_t REPLY_QUEUE_LEN = 16;
constexpr uint64_t STREAM_INTERVAL_US = 4000;

struct reply_t {
    uint8_t id;
    uint8_t payload[REPORT_LEN];
};

reply_t replies[REPLY_QUEUE_LEN];
uint8_t reply_head = 0;
uint8_t reply_tail = 0;
uint8_t reply_count = 0;

uint8_t horipad[8] = { 0, 0, 0x0F, 0x80, 0x80, 0x80, 0x80, 0 };
int16_t imu[6] = { 0, 0, 16384, 0, 0, 0 };
uint8_t user_cal[256];
bool user_cal_initialized = false;
uint64_t last_imu_us = 0;
uint64_t last_stream_us = 0;
uint8_t timer_byte = 0;
uint8_t input_mode = 0;
bool imu_enabled = false;

void enqueue(uint8_t id, const uint8_t* payload, uint8_t len) {
    if (reply_count == REPLY_QUEUE_LEN) {
        return;
    }
    reply_t& reply = replies[reply_tail];
    reply.id = id;
    memset(reply.payload, 0, sizeof(reply.payload));
    memcpy(reply.payload, payload, len);
    reply_tail = (reply_tail + 1) % REPLY_QUEUE_LEN;
    reply_count++;
}

void device_mac(uint8_t* mac) {
    uint64_t unique_id = get_unique_id();
    mac[0] = 0x02;  // Locally administered, unicast
    for (uint8_t i = 1; i < 6; i++) {
        mac[i] = (unique_id >> (8 * (i - 1))) & 0xFF;
    }
}

uint16_t stick_axis(uint8_t value, bool invert) {
    int32_t centered = (invert ? 128 - value : value - 128) * 16 + 2048;
    if (centered < 0) return 0;
    if (centered > 4095) return 4095;
    return centered;
}

void pack_stick(uint8_t* dst, uint8_t x, uint8_t y) {
    uint16_t xx = stick_axis(x, false);
    uint16_t yy = stick_axis(y, true);
    dst[0] = xx & 0xFF;
    dst[1] = (xx >> 8) | ((yy & 0x0F) << 4);
    dst[2] = yy >> 4;
}

uint32_t buttons_from_horipad() {
    uint16_t buttons = horipad[0] | (uint16_t(horipad[1]) << 8);
    uint32_t out = 0;
    constexpr uint8_t bit_map[14] = {
        0, 2, 3, 1, 22, 6, 23, 7, 8, 9, 11, 10, 12, 13
    };
    for (uint8_t i = 0; i < 14; i++) {
        if (buttons & (1u << i)) out |= 1u << bit_map[i];
    }
    uint8_t hat = horipad[2] & 0x0F;
    if (hat == 0 || hat == 1 || hat == 7) out |= 1u << 17;  // Up
    if (hat == 1 || hat == 2 || hat == 3) out |= 1u << 18;  // Right
    if (hat == 3 || hat == 4 || hat == 5) out |= 1u << 16;  // Down
    if (hat == 5 || hat == 6 || hat == 7) out |= 1u << 19;  // Left
    return out;
}

void input_prefix(uint8_t* out) {
    out[0] = timer_byte++;
    out[1] = 0x80;  // Full battery, not charging
    uint32_t buttons = buttons_from_horipad();
    out[2] = buttons;
    out[3] = buttons >> 8;
    out[4] = buttons >> 16;
    pack_stick(out + 5, horipad[3], horipad[4]);
    pack_stick(out + 8, horipad[5], horipad[6]);
    out[11] = 0x09;
}

int16_t clamp16(int32_t value) {
    if (value < -32768) return -32768;
    if (value > 32767) return 32767;
    return value;
}

void put16(uint8_t* dst, int16_t value) {
    dst[0] = value & 0xFF;
    dst[1] = (uint16_t(value) >> 8) & 0xFF;
}

void full_input(uint8_t* out) {
    memset(out, 0, REPORT_LEN);
    input_prefix(out);
    if (!imu_enabled) return;

    // Steam Controller: X/Y/Z acceleration and angular velocity. Rotate both
    // vectors into the Switch frame. Its accelerometer expects about 4096/g.
    bool fresh = last_imu_us && get_time() - last_imu_us < 100000;
    int32_t ax = fresh ? imu[0] : 0;
    int32_t ay = fresh ? imu[1] : 0;
    int32_t az = fresh ? imu[2] : 16384;
    int32_t gx = fresh ? imu[3] : 0;
    int32_t gy = fresh ? imu[4] : 0;
    int32_t gz = fresh ? imu[5] : 0;
    int16_t sample[6] = {
        clamp16(ay / 4), clamp16(-ax / 4), clamp16(az / 4),
        clamp16(gy * 4 / 5), clamp16(-gx * 9 / 10), clamp16(gz * 9 / 10),
    };
    for (uint8_t n = 0; n < 3; n++) {
        for (uint8_t axis = 0; axis < 6; axis++) {
            put16(out + 12 + n * 12 + axis * 2, sample[axis]);
        }
    }
}

void pack12(uint8_t* dst, const uint16_t* values) {
    for (uint8_t i = 0; i < 3; i++) {
        dst[i * 3] = values[i * 2] & 0xFF;
        dst[i * 3 + 1] = (values[i * 2] >> 8) | (values[i * 2 + 1] << 4);
        dst[i * 3 + 2] = values[i * 2 + 1] >> 4;
    }
}

uint8_t spi_byte(uint32_t address) {
    static const uint8_t imu_cal[24] = {
        0, 0, 0, 0, 0, 0, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
        0, 0, 0, 0, 0, 0, 0x3B, 0x34, 0x3B, 0x34, 0x3B, 0x34,
    };
    static const uint8_t stick_params[24] = {
        0x50, 0xFD, 0x00, 0x00, 0xC6, 0x0F, 0x0F, 0x30,
        0x61, 0x96, 0x30, 0xF3, 0xD4, 0x14, 0x54, 0x41,
        0x15, 0x54, 0xC7, 0x79, 0x9C, 0x33, 0x36, 0x63,
    };
    if (address >= 0x6020 && address < 0x6038) return imu_cal[address - 0x6020];
    if (address >= 0x603D && address < 0x604F) {
        const uint16_t left[6] = { 1800, 1800, 2048, 2048, 1800, 1800 };
        const uint16_t right[6] = { 2048, 2048, 1800, 1800, 1800, 1800 };
        uint8_t packed[18];
        pack12(packed, left);
        pack12(packed + 9, right);
        return packed[address - 0x603D];
    }
    if (address >= 0x6050 && address < 0x605D) return address == 0x605C ? 0xFF : 0x32;
    if (address >= 0x6080 && address < 0x6098) return stick_params[address - 0x6080];
    if (address >= 0x6098 && address < 0x60AA) return stick_params[address - 0x6098 + 6];
    if (address >= 0x8000 && address < 0x8100) return user_cal[address - 0x8000];
    return 0xFF;
}

void subcommand(uint8_t sub, const uint8_t* args, uint16_t len) {
    uint8_t reply[REPORT_LEN] = {};
    input_prefix(reply);
    reply[12] = 0x80;
    reply[13] = sub;
    switch (sub) {
        case 0x01: {  // Manual pairing
            reply[12] = 0x81;
            uint8_t stage = len ? args[0] : 3;
            reply[14] = stage;
            if (stage == 1) {
                device_mac(reply + 15);
                reply[22] = 0x25;
                reply[23] = 0x08;
                memcpy(reply + 24, "Pro Controller", 14);
                reply[43] = 0x68;
            } else if (stage == 2) {
                uint8_t mac[6];
                device_mac(mac);
                for (uint8_t i = 0; i < 16; i++) reply[15 + i] = mac[i % 6] ^ (i * 37);
            }
            break;
        }
        case 0x02:  // Device info
            reply[12] = 0x82;
            reply[14] = 0x03;
            reply[15] = 0x48;
            reply[16] = 0x03;
            reply[17] = 0x02;
            device_mac(reply + 18);
            reply[24] = 0x01;
            reply[25] = 0x01;
            break;
        case 0x03:  // Input report mode
            if (len) input_mode = args[0];
            break;
        case 0x04:  // Trigger elapsed times
            reply[12] = 0x83;
            reply[15] = 0xCC;
            reply[17] = 0xEE;
            reply[19] = 0xFF;
            break;
        case 0x10: {  // SPI flash read
            if (len < 5) break;
            uint32_t address = uint32_t(args[0]) | (uint32_t(args[1]) << 8) |
                               (uint32_t(args[2]) << 16) | (uint32_t(args[3]) << 24);
            uint8_t read_len = args[4] < 29 ? args[4] : 29;
            reply[12] = 0x90;
            memcpy(reply + 14, args, 4);
            reply[18] = read_len;
            for (uint8_t i = 0; i < read_len; i++) reply[19 + i] = spi_byte(address + i);
            break;
        }
        case 0x11: {  // SPI user calibration write (kept until power is removed)
            if (len < 5) break;
            uint32_t address = uint32_t(args[0]) | (uint32_t(args[1]) << 8) |
                               (uint32_t(args[2]) << 16) | (uint32_t(args[3]) << 24);
            uint16_t write_len = args[4];
            if (write_len > len - 5) write_len = len - 5;
            for (uint16_t i = 0; i < write_len; i++) {
                if (address + i >= 0x8000 && address + i < 0x8100) {
                    user_cal[address + i - 0x8000] = args[5 + i];
                }
            }
            break;
        }
        case 0x40:  // Enable IMU
            if (len) imu_enabled = args[0] != 0;
            break;
        case 0x21:  // NFC/IR configuration
            reply[12] = 0xA0;
            break;
        default:
            break;
    }
    enqueue(0x21, reply, REPORT_LEN);
}

}  // namespace

void switch_pro_reset() {
    reply_head = reply_tail = reply_count = 0;
    input_mode = 0;
    imu_enabled = false;
    last_stream_us = 0;
    timer_byte = 0;
    if (!user_cal_initialized) {
        memset(user_cal, 0xFF, sizeof(user_cal));
        user_cal_initialized = true;
    }
}

void switch_pro_update_horipad(const uint8_t* report, uint16_t len) {
    if (len >= sizeof(horipad)) memcpy(horipad, report, sizeof(horipad));
}

void switch_pro_imu_input(uint32_t usage, int32_t value) {
    if (usage >= 0x00200453 && usage <= 0x00200455) {
        imu[usage - 0x00200453] = clamp16(value);
    } else if (usage >= 0x00200457 && usage <= 0x00200459) {
        imu[usage - 0x00200457 + 3] = clamp16(value);
    } else {
        return;
    }
    last_imu_us = get_time();
}

void switch_pro_handle_set_report(uint8_t report_id, const uint8_t* buffer, uint16_t len) {
    if (!report_id && len) {
        report_id = *buffer++;
        len--;
    }
    if (report_id == 0x80 && len) {
        uint8_t reply[9] = { buffer[0] };
        if (buffer[0] == 0x01) {
            reply[2] = 0x03;  // Pro Controller
            device_mac(reply + 3);
            enqueue(0x81, reply, sizeof(reply));
        } else if (buffer[0] == 0x02 || buffer[0] == 0x03) {
            enqueue(0x81, reply, 1);
        }
    } else if (report_id == 0x01 && len >= 10) {
        subcommand(buffer[9], buffer + 10, len - 10);
    }
}

void switch_pro_task() {
    if (!tud_hid_n_ready(0)) return;
    if (reply_count) {
        reply_t& reply = replies[reply_head];
        if (tud_hid_n_report(0, reply.id, reply.payload, REPORT_LEN)) {
            reply_head = (reply_head + 1) % REPLY_QUEUE_LEN;
            reply_count--;
        }
        return;
    }
    if (input_mode != 0x30) return;
    uint64_t now = get_time();
    if (now - last_stream_us < STREAM_INTERVAL_US) return;
    uint8_t report[REPORT_LEN];
    full_input(report);
    if (tud_hid_n_report(0, 0x30, report, sizeof(report))) last_stream_us = now;
}
