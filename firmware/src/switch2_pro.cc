#include <cstring>

#include <tusb.h>

#include "aes128.h"
#include "platform.h"
#include "switch2_pro.h"
#include "switch_pro.h"

// Input reports 0x05 and 0x09 (63 bytes each) and output report 0x02
// (rumble), as a real Switch 2 Pro Controller describes them.
const uint8_t switch2_pro_report_descriptor[] = {
    0x05, 0x01, 0x09, 0x05, 0xA1, 0x01,       // Game Pad application
    0x85, 0x05, 0x05, 0xFF, 0x09, 0x01,       // Report 0x05, vendor data
    0x15, 0x00, 0x26, 0xFF, 0x00,
    0x95, 0x3F, 0x75, 0x08, 0x81, 0x02,
    0x85, 0x09, 0x09, 0x01,                   // Report 0x09
    0x95, 0x02, 0x81, 0x02,                   // Counter, power
    0x05, 0x09, 0x19, 0x01, 0x29, 0x15,
    0x25, 0x01, 0x95, 0x15, 0x75, 0x01,
    0x81, 0x02,                               // 21 buttons
    0x95, 0x01, 0x75, 0x03, 0x81, 0x03,
    0x05, 0x01, 0x09, 0x01, 0xA1, 0x00,
    0x09, 0x30, 0x09, 0x31, 0x09, 0x33, 0x09, 0x35,
    0x26, 0xFF, 0x0F, 0x95, 0x04, 0x75, 0x0C,
    0x81, 0x02,                               // Packed sticks
    0xC0,
    0x05, 0xFF, 0x09, 0x02, 0x26, 0xFF, 0x00,
    0x95, 0x34, 0x75, 0x08, 0x81, 0x02,       // Motion and the rest
    0x85, 0x02, 0x09, 0x01, 0x95, 0x3F,
    0x91, 0x02,                               // Output report 0x02
    0xC0,
};

static_assert(sizeof(switch2_pro_report_descriptor) == switch2_pro_report_descriptor_length,
              "Update the Switch 2 Pro HID descriptor length");

namespace {

constexpr uint64_t REPORT_INTERVAL_US = 4000;
constexpr uint8_t VENDOR_ITF = 0;

// Command header: command, direction, transport, subcommand, unknown,
// data length (request) or acknowledgement (reply), 2 reserved bytes.
constexpr uint8_t HEADER_LEN = 8;
constexpr uint8_t DIRECTION_REPLY = 0x01;
constexpr uint8_t TRANSPORT_USB = 0x00;

constexpr uint8_t FEATURE_BUTTONS = 0x01;
constexpr uint8_t FEATURE_STICKS = 0x02;
constexpr uint8_t FEATURE_MOTION = 0x04;
constexpr uint8_t FEATURE_MOUSE = 0x10;
constexpr uint8_t FEATURE_RUMBLE = 0x20;
constexpr uint8_t FEATURE_MAGNETOMETER = 0x80;

// Firmware 2.1.4, Pro Controller, Bluetooth patch 0.0.12, DSP 0.2.3.
const uint8_t firmware_version[12] = { 2, 1, 4, 2, 0, 0, 12, 0, 0, 2, 3, 0 };

// Pairing: the device half of the key is fixed on real controllers.
const uint8_t device_key[16] = {
    0x5C, 0xF6, 0xEE, 0x79, 0x2C, 0xDF, 0x05, 0xE1,
    0xBA, 0x2B, 0x63, 0x25, 0xC4, 0x1A, 0x5F, 0x10,
};

// Factory data the console reads from controller flash. Sticks are
// uncalibrated 12-bit values centred at 0x800; the calibration declares
// +/-1800 so a full throw of the 8-bit source saturates.
const uint8_t stick_parameters[40] = {
    0x01, 0xAD, 0xD9, 0x9A, 0x55, 0x56, 0x65, 0xA0, 0x00, 0x0A,
    0xA0, 0x00, 0x0A, 0xE2, 0x20, 0x0E, 0xE2, 0x20, 0x0E, 0x9A,
    0xAD, 0xD9, 0x9A, 0xAD, 0xD9, 0x0A, 0xA5, 0x50, 0x0A, 0xA5,
    0x50, 0x2F, 0xF6, 0x62, 0x2F, 0xF6, 0x62, 0x0A, 0xFF, 0xFF,
};
const uint8_t stick_calibration[9] = {
    0x00, 0x08, 0x80,  // Centre 0x800, 0x800
    0x08, 0x87, 0x70,  // Range 1800, 1800
    0x08, 0x87, 0x70,
};
const uint8_t factory_ids[37] = {
    0x01, 0x00,
    'H', 'R', 'M', '2', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1', 0x00, 0x00,
    0x7E, 0x05, 0x69, 0x20,  // VID, PID
    0x01, 0x06, 0x01,
    0x23, 0x23, 0x23, 0xA0, 0xA0, 0xA0,  // Body, buttons
    0xE6, 0xE6, 0xE6, 0x32, 0x32, 0x32,  // Highlight, grip
};
const uint8_t motion_calibration[16] = {
    0x00, 0x00, 0xC8, 0x41,  // 25.0 degrees C (float)
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // Gyro bias
};
const uint8_t sensor_bias[24] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // Magnetometer
    0, 0, 0, 0, 0, 0, 0, 0,
    0xC3, 0xF5, 0x1C, 0x41,  // Accelerometer Z: 9.81 (float)
};
const uint8_t factory_serial[17] = { '1', '0', '0', '0', '0', '0', '0', '0', 'L', 'D', '1', '0', '0', '0', '0', '0', '0' };
const uint8_t factory_e20[4] = { 0x01, 0x02, 0x02, 0x02 };
const uint8_t factory_e30[10] = { 0x03, 0x02, 0x04, 0x01, 0x05, 0x02, 0x06, 0x02, 0x07, 0x02 };
const uint8_t factory_e60[2] = { 0x11, 0x00 };
const uint8_t factory_e80[9] = { 0x00, 0x23, 0xE9, 0xCE, 0x00, 0x41, 0x30, 0x4B, 0x41 };
const uint8_t factory_efb[4] = { 0x01, 0x00, 0x15, 0x0C };

struct flash_region_t {
    uint32_t address;
    const uint8_t* data;
    uint8_t len;
};

const flash_region_t flash_regions[] = {
    { 0x13000, factory_ids, sizeof(factory_ids) },
    { 0x13040, motion_calibration, sizeof(motion_calibration) },
    { 0x13080, stick_parameters, sizeof(stick_parameters) },
    { 0x130A8, stick_calibration, sizeof(stick_calibration) },
    { 0x130C0, stick_parameters, sizeof(stick_parameters) },
    { 0x130E8, stick_calibration, sizeof(stick_calibration) },
    { 0x13100, sensor_bias, sizeof(sensor_bias) },
    { 0x13E00, factory_serial, sizeof(factory_serial) },
    { 0x13E20, factory_e20, sizeof(factory_e20) },
    { 0x13E30, factory_e30, sizeof(factory_e30) },
    { 0x13E60, factory_e60, sizeof(factory_e60) },
    { 0x13E80, factory_e80, sizeof(factory_e80) },
    { 0x13EFB, factory_efb, sizeof(factory_efb) },
};

// Report 0x30 button bit -> report 0x09 button bit.
const uint8_t report_09_buttons[][2] = {
    { 0, 2 }, { 1, 3 }, { 2, 0 }, { 3, 1 },       // Y, X, B, A
    { 6, 4 }, { 7, 5 }, { 8, 14 }, { 9, 6 },      // R, ZR, Minus, Plus
    { 10, 7 }, { 11, 15 }, { 12, 16 }, { 13, 17 },  // R3, L3, Home, Capture
    { 16, 8 }, { 17, 11 }, { 18, 9 }, { 19, 10 },  // Down, Up, Right, Left
    { 22, 12 }, { 23, 13 },                        // L, ZL
};
constexpr uint8_t REPORT_09_GR = 18;
constexpr uint8_t REPORT_09_GL = 19;
constexpr uint8_t REPORT_05_GR = 24;
constexpr uint8_t REPORT_05_GL = 25;

bool usb_initialised = false;
uint8_t report_id = 0x09;
uint8_t feature_mask = 0;
uint8_t features_enabled = 0;
uint32_t counter = 0;
uint64_t last_report_us = 0;
uint8_t host_key[16];
uint8_t rx[256];
uint16_t rx_len = 0;
uint8_t reply_buf[SWITCH2_PRO_MAX_REPLY];

void put16(uint8_t* dst, uint16_t value) {
    dst[0] = value & 0xFF;
    dst[1] = value >> 8;
}

void put32(uint8_t* dst, uint32_t value) {
    for (uint8_t i = 0; i < 4; i++) dst[i] = value >> (8 * i);
}

uint32_t get32(const uint8_t* src) {
    return uint32_t(src[0]) | (uint32_t(src[1]) << 8) | (uint32_t(src[2]) << 16) | (uint32_t(src[3]) << 24);
}

void pack_stick(uint8_t* dst, uint16_t x, uint16_t y) {
    dst[0] = x & 0xFF;
    dst[1] = (x >> 8) | ((y & 0x0F) << 4);
    dst[2] = y >> 4;
}

// Bluetooth address, most significant byte first.
void device_address(uint8_t* address) {
    uint64_t unique_id = get_unique_id();
    for (uint8_t i = 0; i < 6; i++) address[i] = unique_id >> (8 * (5 - i));
}

void device_address_reversed(uint8_t* out) {
    uint8_t address[6];
    device_address(address);
    for (uint8_t i = 0; i < 6; i++) out[i] = address[5 - i];
}

void read_flash(uint32_t address, uint8_t* out, uint8_t len) {
    memset(out, 0xFF, len);
    for (auto const& region : flash_regions) {
        for (uint8_t i = 0; i < region.len; i++) {
            uint32_t a = region.address + i;
            if (a >= address && a < address + len) out[a - address] = region.data[i];
        }
    }
}

// Each handler writes its reply data and returns its length.
uint16_t flash_command(uint8_t sub, const uint8_t* data, uint8_t len, uint8_t* out) {
    memset(out, 0, 8);
    switch (sub) {
        case 0x01:  // Read 64-byte block
            if (len < 8) return 0;
            out[0] = 64;
            memcpy(out + 4, data + 4, 4);
            read_flash(get32(data + 4), out + 8, 64);
            return 8 + 64;
        case 0x04: {  // Read
            if (len < 8) return 0;
            uint8_t read_len = data[0] < 80 ? data[0] : 80;
            out[0] = read_len;
            memcpy(out + 4, data + 4, 4);
            read_flash(get32(data + 4), out + 8, read_len);
            return 8 + read_len;
        }
        case 0x02:  // Write block
        case 0x05:  // Write
            if (len < 8) return 0;
            memcpy(out + 4, data + 4, 4);
            return 8;
        case 0x03:  // Erase sector
            return 4;
        default:
            return 0;
    }
}

uint16_t init_command(uint8_t sub, const uint8_t* data, uint8_t len, uint8_t* out) {
    memset(out, 0, 4);
    switch (sub) {
        case 0x03:  // Enable USB HID reports
        case 0x0D:  // Initialise USB
            if (sub == 0x0D) usb_initialised = true;
            out[0] = 0x01;
            return 4;
        case 0x0A:  // Select input report
            if (len >= 1 && (data[0] == 0x05 || data[0] == 0x09)) report_id = data[0];
            return 0;
        case 0x0F:
            out[0] = 0x05;
            return 4;
        default:
            return 0;
    }
}

uint16_t feature_command(uint8_t sub, const uint8_t* data, uint8_t len, uint8_t* out) {
    uint8_t flags = len ? data[0] : 0;
    memset(out, 0, 12);
    switch (sub) {
        case 0x01:  // Feature info
            out[4] = flags & FEATURE_BUTTONS ? 0x07 : 0;
            out[5] = flags & FEATURE_STICKS ? 0x07 : 0;
            out[6] = flags & FEATURE_MOTION ? 0x01 : 0;
            out[7] = flags & FEATURE_MAGNETOMETER ? 0x01 : 0;
            out[8] = flags & FEATURE_MOUSE ? 0x01 : 0;
            out[9] = flags & FEATURE_RUMBLE ? 0x03 : 0;
            return 12;
        case 0x02:
            feature_mask = flags;
            return 4;
        case 0x03:
            feature_mask = 0;
            return 4;
        case 0x04:
            features_enabled |= flags;
            return 4;
        case 0x05:
            features_enabled &= ~flags;
            return 4;
        default:
            return 0;
    }
}

uint16_t pairing_command(uint8_t sub, const uint8_t* data, uint8_t len, uint8_t* out) {
    switch (sub) {
        case 0x01:  // Exchange addresses
            out[0] = 0x01;
            out[1] = 0x04;
            out[2] = 0x01;
            device_address_reversed(out + 3);
            return 9;
        case 0x02: {  // Confirm LTK: AES(reverse(A1 ^ B1), reverse(challenge))
            if (len < 17) return 0;
            uint8_t ltk[16];
            uint8_t challenge[16];
            for (uint8_t i = 0; i < 16; i++) {
                ltk[i] = host_key[15 - i] ^ device_key[15 - i];
                challenge[i] = data[16 - i];
            }
            out[0] = 0x01;
            aes128_encrypt(ltk, challenge, out + 1);
            return 17;
        }
        case 0x03:  // Finalise
            out[0] = 0x01;
            return 1;
        case 0x04:  // Exchange keys
            if (len < 17) return 0;
            memcpy(host_key, data + 1, 16);
            out[0] = 0x01;
            memcpy(out + 1, device_key, 16);
            return 17;
        default:
            return 0;
    }
}

// Replies that real controllers give to commands whose purpose is unknown.
uint16_t other_command(uint8_t command, uint8_t sub, const uint8_t* data, uint8_t len, uint8_t* out) {
    static const uint8_t reply_11_03[29] = {
        0x01, 0x20, 0x03, 0x00, 0x00, 0x0A, 0xE8, 0x1C, 0x3B, 0x79,
        0x7D, 0x8B, 0x3A, 0x0A, 0xE8, 0x9C, 0x42, 0x58, 0xA0, 0x0B,
        0x42, 0x0A, 0xE8, 0x9C, 0x41, 0x58, 0xA0, 0x0B, 0x41,
    };
    static const uint8_t reply_18_01[8] = { 0x00, 0x00, 0x40, 0xF0, 0x00, 0x00, 0x60, 0x00 };
    memset(out, 0, 24);
    switch ((command << 8) | sub) {
        case 0x010C:
            out[0] = 0x61;
            out[1] = 0x12;
            out[2] = 0x50;
            out[3] = 0x0D;
            return 4;
        case 0x0701:
            return 1;
        case 0x0B03:  // Battery voltage: 3749 mV
            put16(out, 0x0EA5);
            return 4;
        case 0x0B04:  // Charge status
            out[0] = 0x34;
            out[2] = 0x83;
            return 4;
        case 0x0B06:
            out[0] = 0x11;
            return 4;
        case 0x1001:  // Firmware version
            memcpy(out, firmware_version, sizeof(firmware_version));
            return sizeof(firmware_version);
        case 0x1101:
        case 0x1301:
            out[0] = 0x01;
            return 4;
        case 0x1103:
            memcpy(out, reply_11_03, sizeof(reply_11_03));
            return sizeof(reply_11_03);
        case 0x1302:
        case 0x1303:
            out[0] = 0x01;
            return 8;
        case 0x1601:
            return 24;
        case 0x1801:
            memcpy(out, reply_18_01, sizeof(reply_18_01));
            return sizeof(reply_18_01);
        case 0x1803:
            out[0] = len ? data[0] : 0;
            return 1;
        default:
            return 0;
    }
}

uint32_t report_09_bits(const switch_pro_input_t& input) {
    uint32_t out = 0;
    for (auto const& map : report_09_buttons) {
        if (input.buttons & (1u << map[0])) out |= 1u << map[1];
    }
    if (input.back_right) out |= 1u << REPORT_09_GR;
    if (input.back_left) out |= 1u << REPORT_09_GL;
    return out;
}

}  // namespace

void switch2_pro_reset() {
    usb_initialised = false;
    report_id = 0x09;
    feature_mask = 0;
    features_enabled = 0;
    counter = 0;
    last_report_us = 0;
    memset(host_key, 0, sizeof(host_key));
    rx_len = 0;
}

uint16_t switch2_pro_command(const uint8_t* request, uint16_t len, uint8_t* reply) {
    if (len < HEADER_LEN) return 0;
    uint8_t command = request[0];
    uint8_t sub = request[3];
    uint8_t transport = request[2];
    const uint8_t* data = request + HEADER_LEN;
    uint8_t data_len = request[5];
    if (data_len > len - HEADER_LEN) data_len = len - HEADER_LEN;

    reply[0] = command;
    reply[1] = DIRECTION_REPLY;
    reply[2] = transport;
    reply[3] = sub;
    // Acknowledgement: 0xF800 over USB, 0x7810 over Bluetooth.
    reply[4] = transport == TRANSPORT_USB ? 0x00 : 0x10;
    reply[5] = transport == TRANSPORT_USB ? 0xF8 : 0x78;
    reply[6] = reply[7] = 0;

    uint8_t* out = reply + HEADER_LEN;
    uint16_t out_len;
    switch (command) {
        case 0x02:
            out_len = flash_command(sub, data, data_len, out);
            break;
        case 0x03:
            out_len = init_command(sub, data, data_len, out);
            break;
        case 0x0C:
            out_len = feature_command(sub, data, data_len, out);
            break;
        case 0x15:
            out_len = pairing_command(sub, data, data_len, out);
            break;
        default:
            out_len = other_command(command, sub, data, data_len, out);
            break;
    }
    return HEADER_LEN + out_len;
}

int32_t switch2_pro_vendor_request(uint8_t request, uint8_t* data) {
    switch (request) {
        case 2:  // Device info: firmware version and Bluetooth address
            memcpy(data, firmware_version, sizeof(firmware_version));
            device_address_reversed(data + sizeof(firmware_version));
            return SWITCH2_PRO_DEVICE_INFO_LEN;
        case 3:  // Factory data
            read_flash(0x13000, data, SWITCH2_PRO_FACTORY_DATA_LEN);
            return SWITCH2_PRO_FACTORY_DATA_LEN;
        default:
            return -1;
    }
}

uint8_t switch2_pro_build_report(uint8_t* out) {
    switch_pro_input_t input;
    switch_pro_get_input(&input);
    memset(out, 0, SWITCH2_PRO_REPORT_LEN);
    if (report_id == 0x05) {
        uint32_t buttons = input.buttons & 0x00FFFFFF;
        if (input.back_right) buttons |= 1u << REPORT_05_GR;
        if (input.back_left) buttons |= 1u << REPORT_05_GL;
        put32(out, counter);
        put32(out + 0x04, buttons);
        pack_stick(out + 0x0A, input.sticks[0], input.sticks[1]);
        pack_stick(out + 0x0D, input.sticks[2], input.sticks[3]);
        put16(out + 0x1F, 0x0EA5);  // Battery voltage
        out[0x21] = 0x20;           // Charged
        out[0x29] = 0x01;
        return 0x05;
    }
    uint32_t buttons = report_09_bits(input);
    out[0] = counter;
    out[1] = 0x25;  // External power, battery level 9
    out[2] = buttons;
    out[3] = buttons >> 8;
    out[4] = buttons >> 16;
    pack_stick(out + 5, input.sticks[0], input.sticks[1]);
    pack_stick(out + 8, input.sticks[2], input.sticks[3]);
    out[0x0B] = feature_mask & FEATURE_RUMBLE ? 0x38 : 0x30;
    return 0x09;
}

void switch2_pro_handle_set_report(uint8_t, const uint8_t*, uint16_t) {
    // Output report 0x02 carries rumble, which has nowhere to go.
}

void switch2_pro_task() {
    // Commands can span several bulk packets; the header gives the length.
    if (rx_len < sizeof(rx) && tud_vendor_n_available(VENDOR_ITF)) {
        rx_len += tud_vendor_n_read(VENDOR_ITF, rx + rx_len, sizeof(rx) - rx_len);
    }
    if (rx_len >= HEADER_LEN) {
        uint16_t need = HEADER_LEN + rx[5];
        if (need > sizeof(rx)) {
            rx_len = 0;  // Cannot be a command we know; resynchronise
        } else if (rx_len >= need && tud_vendor_n_write_available(VENDOR_ITF) >= SWITCH2_PRO_MAX_REPLY) {
            uint16_t reply_len = switch2_pro_command(rx, need, reply_buf);
            tud_vendor_n_write(VENDOR_ITF, reply_buf, reply_len);
            tud_vendor_n_write_flush(VENDOR_ITF);
            memmove(rx, rx + need, rx_len - need);
            rx_len -= need;
        }
    }

    if (!usb_initialised || !feature_mask || !tud_hid_n_ready(0)) return;
    uint64_t now = get_time();
    if (now - last_report_us < REPORT_INTERVAL_US) return;
    uint8_t report[SWITCH2_PRO_REPORT_LEN];
    uint8_t id = switch2_pro_build_report(report);
    if (tud_hid_n_report(0, id, report, sizeof(report))) {
        last_report_us = now;
        counter++;
    }
}
