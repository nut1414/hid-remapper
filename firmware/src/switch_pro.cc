#include <cstring>

#include <tusb.h>

#include "crc.h"
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
// A button or d-pad change may pre-empt the stream once the previous report
// has had a USB frame to be read. The stream then restarts from that report,
// so the steady report rate (which motion is tuned for) stays the same. The
// console integrates gyro per report rather than per elapsed time, so an early
// report scales its gyro by the time it actually covers.
constexpr uint64_t BUTTON_MIN_INTERVAL_US = 1000;
// The console writes calibration in bursts; save once it has been quiet.
constexpr uint64_t CAL_SAVE_DELAY_US = 250000;
constexpr uint32_t CAL_MAGIC = 0x31435053;  // "SPC1"

// Touchpads: axes span +/-32767 and read 0,0 when untouched. They have no
// click switch, so a click is pressure crossing a threshold (with hysteresis).
constexpr int32_t PAD_PRESS = 4000;
constexpr int32_t PAD_RELEASE = 2500;
// Right pad movement is added to the gyro as rotation: one gyro count per
// this many pad units.
constexpr int32_t PAD_AIM_DIVISOR = 1;
// Left pad d-pad: how far from the centre a press must be to count.
constexpr int32_t PAD_DPAD_THRESHOLD = 12000;
constexpr uint32_t PRO_ZR = 1u << 7;
constexpr uint32_t PRO_R3 = 1u << 10;
constexpr uint32_t PRO_L3 = 1u << 11;
constexpr uint32_t PRO_DOWN = 1u << 16;
constexpr uint32_t PRO_UP = 1u << 17;
constexpr uint32_t PRO_RIGHT = 1u << 18;
constexpr uint32_t PRO_LEFT = 1u << 19;

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
bool user_cal_dirty = false;
uint64_t user_cal_changed_us = 0;
uint8_t cal_image[SWITCH_PRO_CAL_IMAGE_SIZE];
uint64_t last_imu_us = 0;
uint64_t last_stream_us = 0;
uint8_t timer_byte = 0;
uint8_t input_mode = 0;
bool imu_enabled = false;
uint32_t sent_buttons = 0;
bool buttons_changed = false;

// Latency counters, read and cleared through the config interface. Times are
// measured from a host input report arriving to the Pro report that carries
// it being read by the console.
switch_pro_latency_stats_t stats;
uint64_t last_input_us = 0;
uint64_t pending_input_us = 0;
uint64_t pending_button_us = 0;
uint64_t in_flight_input_us = 0;
uint64_t in_flight_button_us = 0;

struct pad_t {
    int32_t x, y, pressure;
    int32_t prev_x, prev_y;
    bool touching;
    bool pressed;
};
pad_t left_pad;
pad_t right_pad;
uint32_t pad_buttons = 0;
bool back_buttons[4] = {};  // L4, R4, L5, R5
int32_t aim_x = 0;  // Right pad movement not yet sent as rotation
int32_t aim_y = 0;

uint16_t clamp_us(uint64_t value) {
    return value > 0xFFFF ? 0xFFFF : value;
}

void record_latency(uint64_t since, uint64_t now, uint16_t& count, uint32_t& sum, uint16_t& max) {
    if (!since) return;
    uint16_t latency = clamp_us(now - since);
    if (count < 0xFFFF) count++;
    sum += latency;
    if (latency > max) max = latency;
}

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

// Back buttons L5 and R5 click the sticks.
uint32_t back_stick_clicks() {
    return (back_buttons[2] ? PRO_L3 : 0) | (back_buttons[3] ? PRO_R3 : 0);
}

uint32_t current_buttons() {
    return buttons_from_horipad() | pad_buttons | back_stick_clicks();
}

void update_pad(pad_t& pad) {
    pad.touching = pad.x || pad.y;
    if (pad.pressure >= PAD_PRESS) pad.pressed = true;
    if (pad.pressure <= PAD_RELEASE) pad.pressed = false;
}

uint32_t left_pad_dpad() {
    if (!left_pad.pressed || !left_pad.touching) return 0;
    uint32_t out = 0;
    if (left_pad.y > PAD_DPAD_THRESHOLD) out |= PRO_UP;
    if (left_pad.y < -PAD_DPAD_THRESHOLD) out |= PRO_DOWN;
    if (left_pad.x > PAD_DPAD_THRESHOLD) out |= PRO_RIGHT;
    if (left_pad.x < -PAD_DPAD_THRESHOLD) out |= PRO_LEFT;
    return out;
}

void input_prefix(uint8_t* out) {
    out[0] = timer_byte++;
    out[1] = 0x80;  // Full battery, not charging
    uint32_t buttons = current_buttons();
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

// Adds as much pending pad movement to a gyro sample as fits in 16 bits; the
// rest goes out with the next report.
int16_t add_aim(int32_t base, int32_t& pending, int32_t sign) {
    int32_t counts = sign * (pending / PAD_AIM_DIVISOR);
    int32_t sample = clamp16(base + counts);
    pending -= sign * (sample - base) * PAD_AIM_DIVISOR;
    return sample;
}

void full_input(uint8_t* out, uint64_t covered_us) {
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
    if (covered_us < STREAM_INTERVAL_US) {
        gx = gx * int32_t(covered_us) / int32_t(STREAM_INTERVAL_US);
        gy = gy * int32_t(covered_us) / int32_t(STREAM_INTERVAL_US);
        gz = gz * int32_t(covered_us) / int32_t(STREAM_INTERVAL_US);
    }
    // Pad aim is a distance rather than a rate, so it is not scaled with the
    // time an early report covers. Pad right turns right (negative yaw), pad
    // up looks up (negative pitch).
    int16_t sample[6] = {
        clamp16(ay / 4), clamp16(-ax / 4), clamp16(az / 4),
        clamp16(gy * 4 / 5), 0, 0,
    };
    sample[4] = add_aim(clamp16(-gx * 9 / 10), aim_y, -1);
    sample[5] = add_aim(clamp16(gz * 9 / 10), aim_x, -1);
    for (uint8_t n = 0; n < 3; n++) {
        for (uint8_t axis = 0; axis < 6; axis++) {
            put16(out + 12 + n * 12 + axis * 2, sample[axis]);
        }
    }
}

void load_user_cal() {
    const uint8_t* image = get_persisted_switch_pro_cal();
    uint32_t magic;
    uint32_t crc;
    memcpy(&magic, image, 4);
    memcpy(&crc, image + 4 + sizeof(user_cal), 4);
    if (magic == CAL_MAGIC && crc == crc32(image, 4 + sizeof(user_cal))) {
        memcpy(user_cal, image + 4, sizeof(user_cal));
    }
}

void save_user_cal() {
    memset(cal_image, 0xFF, sizeof(cal_image));
    memcpy(cal_image, &CAL_MAGIC, 4);
    memcpy(cal_image + 4, user_cal, sizeof(user_cal));
    uint32_t crc = crc32(cal_image, 4 + sizeof(user_cal));
    memcpy(cal_image + 4 + sizeof(user_cal), &crc, 4);
    do_persist_switch_pro_cal(cal_image);
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
        case 0x11: {  // SPI user calibration write (saved to flash)
            if (len < 5) break;
            uint32_t address = uint32_t(args[0]) | (uint32_t(args[1]) << 8) |
                               (uint32_t(args[2]) << 16) | (uint32_t(args[3]) << 24);
            uint16_t write_len = args[4];
            if (write_len > len - 5) write_len = len - 5;
            for (uint16_t i = 0; i < write_len; i++) {
                if (address + i >= 0x8000 && address + i < 0x8100 &&
                    user_cal[address + i - 0x8000] != args[5 + i]) {
                    user_cal[address + i - 0x8000] = args[5 + i];
                    user_cal_dirty = true;
                    user_cal_changed_us = get_time();
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
    sent_buttons = 0;
    buttons_changed = false;
    pending_input_us = pending_button_us = 0;
    in_flight_input_us = in_flight_button_us = 0;
    left_pad = right_pad = {};
    pad_buttons = 0;
    memset(back_buttons, 0, sizeof(back_buttons));
    aim_x = aim_y = 0;
    if (!user_cal_initialized) {
        memset(user_cal, 0xFF, sizeof(user_cal));
        load_user_cal();
        user_cal_initialized = true;
    }
}

void switch_pro_update_horipad(const uint8_t* report, uint16_t len) {
    if (len < sizeof(horipad)) return;
    memcpy(horipad, report, sizeof(horipad));
    if (current_buttons() != sent_buttons && !buttons_changed) {
        buttons_changed = true;
        pending_button_us = last_input_us;
    }
}

void switch_pro_input_received() {
    uint64_t now = get_time();
    if (last_input_us) {
        uint16_t gap = clamp_us(now - last_input_us);
        if (gap > stats.input_gap_max_us) stats.input_gap_max_us = gap;
    }
    stats.inputs++;
    last_input_us = now;
    if (!pending_input_us) pending_input_us = now;
}

void switch_pro_input_decoded() {
    update_pad(left_pad);
    // Movement counts only between two touching samples, so a new touch
    // never jumps the aim.
    bool was_touching = right_pad.touching;
    update_pad(right_pad);
    if (right_pad.touching && was_touching) {
        aim_x += right_pad.x - right_pad.prev_x;
        aim_y += right_pad.y - right_pad.prev_y;
    }
    right_pad.prev_x = right_pad.x;
    right_pad.prev_y = right_pad.y;

    pad_buttons = left_pad_dpad() | (right_pad.pressed ? PRO_ZR : 0);
    if (current_buttons() != sent_buttons && !buttons_changed) {
        buttons_changed = true;
        pending_button_us = last_input_us;
    }
}

void switch_pro_get_input(switch_pro_input_t* out) {
    out->buttons = current_buttons();
    out->back_left = back_buttons[0];
    out->back_right = back_buttons[1];
    out->sticks[0] = stick_axis(horipad[3], false);
    out->sticks[1] = stick_axis(horipad[4], true);
    out->sticks[2] = stick_axis(horipad[5], false);
    out->sticks[3] = stick_axis(horipad[6], true);
    out->motion = last_imu_us && get_time() - last_imu_us < 100000;
    for (uint8_t i = 0; i < 3; i++) {
        out->accel[i] = imu[i];
        out->gyro[i] = imu[3 + i];
    }
}

void switch_pro_take_pad_aim(int32_t* x, int32_t* y) {
    *x = aim_x;
    *y = aim_y;
    aim_x = aim_y = 0;
}

void switch_pro_report_complete(uint8_t report_id) {
    if (report_id != 0x30) return;
    uint64_t now = get_time();
    record_latency(in_flight_input_us, now, stats.input_count, stats.input_sum_us, stats.input_max_us);
    record_latency(in_flight_button_us, now, stats.button_count, stats.button_sum_us, stats.button_max_us);
    in_flight_input_us = in_flight_button_us = 0;
}

void switch_pro_get_latency_stats(switch_pro_latency_stats_t* out) {
    *out = stats;
    memset(&stats, 0, sizeof(stats));
}

void switch_pro_raw_input(uint32_t usage, int32_t value) {
    if (usage >= 0x00200453 && usage <= 0x00200455) {
        imu[usage - 0x00200453] = clamp16(value);
    } else if (usage >= 0x00200457 && usage <= 0x00200459) {
        imu[usage - 0x00200457 + 3] = clamp16(value);
    } else if (usage >= SWITCH_PRO_BACK_USAGE_FIRST && usage <= SWITCH_PRO_BACK_USAGE_LAST) {
        back_buttons[usage - SWITCH_PRO_BACK_USAGE_FIRST] = value;
        return;
    } else if (usage >= 0xFFFB0001 && usage <= 0xFFFB0006) {
        pad_t& pad = usage <= 0xFFFB0003 ? left_pad : right_pad;
        int32_t* fields[3] = { &pad.x, &pad.y, &pad.pressure };
        *fields[(usage - 0xFFFB0001) % 3] = value;
        return;
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
    // Flash writes stall the CPU, so wait until replies are out and the
    // console has stopped writing.
    if (user_cal_dirty && !reply_count && get_time() - user_cal_changed_us >= CAL_SAVE_DELAY_US) {
        user_cal_dirty = false;
        save_user_cal();
    }
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
    uint64_t elapsed = now - last_stream_us;
    bool early = buttons_changed && elapsed >= BUTTON_MIN_INTERVAL_US;
    if (elapsed < STREAM_INTERVAL_US && !early) return;
    uint8_t report[REPORT_LEN];
    full_input(report, elapsed);
    if (!tud_hid_n_report(0, 0x30, report, sizeof(report))) return;
    last_stream_us = now;
    sent_buttons = current_buttons();
    buttons_changed = false;
    stats.reports++;
    if (early && elapsed < STREAM_INTERVAL_US) stats.early_reports++;
    in_flight_input_us = pending_input_us;
    in_flight_button_us = pending_button_us;
    pending_input_us = pending_button_us = 0;
}
