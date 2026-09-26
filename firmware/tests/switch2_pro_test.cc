#include <cassert>
#include <cstdint>
#include <cstring>
#include <vector>

#include "switch2_pro.h"
#include "switch_pro.h"

static uint64_t now_us = 0;
static uint8_t last_id = 0;
static uint8_t last_report[63];
static int report_count = 0;
static std::vector<uint8_t> vendor_in;
static std::vector<uint8_t> vendor_out;
static uint8_t flash[SWITCH_PRO_CAL_IMAGE_SIZE];

uint64_t get_time() { return now_us; }
uint64_t get_unique_id() { return 0x123456789ABCDEF0ULL; }
void do_persist_switch_pro_cal(const uint8_t*) {}
const uint8_t* get_persisted_switch_pro_cal() { return flash; }
bool tud_hid_n_ready(uint8_t) { return true; }
bool tud_hid_n_report(uint8_t, uint8_t id, const void* report, uint16_t len) {
    assert(len == 63);
    last_id = id;
    memcpy(last_report, report, len);
    report_count++;
    return true;
}
uint32_t tud_vendor_n_available(uint8_t) { return vendor_in.size(); }
uint32_t tud_vendor_n_read(uint8_t, void* buffer, uint32_t bufsize) {
    uint32_t n = vendor_in.size() < 64 ? vendor_in.size() : 64;  // One bulk packet
    if (n > bufsize) n = bufsize;
    memcpy(buffer, vendor_in.data(), n);
    vendor_in.erase(vendor_in.begin(), vendor_in.begin() + n);
    return n;
}
uint32_t tud_vendor_n_write(uint8_t, void const* buffer, uint32_t bufsize) {
    const uint8_t* p = (const uint8_t*) buffer;
    vendor_out.insert(vendor_out.end(), p, p + bufsize);
    return bufsize;
}
uint32_t tud_vendor_n_write_flush(uint8_t) { return 0; }
uint32_t tud_vendor_n_write_available(uint8_t) { return 256; }

static std::vector<uint8_t> command(std::vector<uint8_t> request) {
    uint8_t reply[SWITCH2_PRO_MAX_REPLY];
    uint16_t len = switch2_pro_command(request.data(), request.size(), reply);
    return std::vector<uint8_t>(reply, reply + len);
}

static std::vector<uint8_t> bytes(std::vector<uint8_t> header, std::vector<uint8_t> data = {}) {
    header.insert(header.end(), data.begin(), data.end());
    return header;
}

int main() {
    memset(flash, 0xFF, sizeof(flash));
    switch_pro_reset();
    switch2_pro_reset();

    // Pairing, checked against a real controller's exchange: the key is
    // reverse(host ^ device key) and the challenge is reversed on the wire.
    auto reply = command(bytes({ 0x15, 0x91, 0x01, 0x04, 0x00, 0x11, 0, 0 },
        { 0x00, 0x35, 0x03, 0xe9, 0x29, 0x82, 0x87, 0x71, 0x24, 0xbe, 0xa8, 0x0c, 0x66, 0x46, 0x15, 0x83, 0x4b }));
    assert(reply == bytes({ 0x15, 0x01, 0x01, 0x04, 0x10, 0x78, 0, 0 },
        { 0x01, 0x5c, 0xf6, 0xee, 0x79, 0x2c, 0xdf, 0x05, 0xe1, 0xba, 0x2b, 0x63, 0x25, 0xc4, 0x1a, 0x5f, 0x10 }));
    reply = command(bytes({ 0x15, 0x91, 0x01, 0x02, 0x00, 0x11, 0, 0 },
        { 0x00, 0x6f, 0xc6, 0xdf, 0x8a, 0xd8, 0xfe, 0xdf, 0x15, 0xbb, 0x8c, 0x15, 0xe9, 0x1f, 0x32, 0x05, 0x44 }));
    assert(reply == bytes({ 0x15, 0x01, 0x01, 0x02, 0x10, 0x78, 0, 0 },
        { 0x01, 0x13, 0x4c, 0x97, 0xf5, 0x11, 0xb9, 0xb6, 0xdd, 0x4d, 0x86, 0xfd, 0x40, 0xf5, 0x36, 0xe9, 0xed }));

    // USB transport acks with 0xF800; unknown commands still get a header.
    reply = command(bytes({ 0x03, 0x91, 0x00, 0x0d, 0x00, 0x08, 0, 0 }, { 0x01, 0x00, 0x31, 0x7e, 0xc6, 0xeb, 0xf1, 0x48 }));
    assert(reply == bytes({ 0x03, 0x01, 0x00, 0x0d, 0x00, 0xf8, 0, 0 }, { 0x01, 0, 0, 0 }));
    reply = command(bytes({ 0x0e, 0x91, 0x00, 0x01, 0x00, 0x00, 0, 0 }));
    assert(reply == bytes({ 0x0e, 0x01, 0x00, 0x01, 0x00, 0xf8, 0, 0 }));
    reply = command(bytes({ 0x0c, 0x91, 0x01, 0x01, 0x00, 0x04, 0, 0 }, { 0x2f, 0, 0, 0 }));
    assert(reply == bytes({ 0x0c, 0x01, 0x01, 0x01, 0x10, 0x78, 0, 0 }, { 0, 0, 0, 0, 0x07, 0x07, 0x01, 0, 0, 0x03, 0, 0 }));
    reply = command(bytes({ 0x10, 0x91, 0x00, 0x01, 0x00, 0x00, 0, 0 }));
    assert(reply.size() == 8 + 12 && reply[8 + 3] == 0x02);  // Pro Controller firmware

    // Factory data: VID/PID at 0x13012, by flash command and vendor request.
    reply = command(bytes({ 0x02, 0x91, 0x00, 0x01, 0x00, 0x08, 0, 0 }, { 0, 0, 0, 0, 0x00, 0x30, 0x01, 0x00 }));
    assert(reply.size() == 8 + 8 + 64 && reply[8] == 64);
    assert(reply[16 + 0x12] == 0x7e && reply[16 + 0x13] == 0x05 && reply[16 + 0x14] == 0x69 && reply[16 + 0x15] == 0x20);
    reply = command(bytes({ 0x02, 0x91, 0x00, 0x04, 0x00, 0x08, 0, 0 }, { 9, 0, 0, 0, 0xa8, 0x30, 0x01, 0x00 }));
    assert(reply.size() == 8 + 8 + 9 && reply[16] == 0x00 && reply[17] == 0x08 && reply[18] == 0x80);
    uint8_t data[64];
    assert(switch2_pro_vendor_request(3, data) == 64 && data[0x14] == 0x69);
    assert(switch2_pro_vendor_request(2, data) == 18 && data[3] == 0x02 && data[12] == 0xF0);
    assert(switch2_pro_vendor_request(9, data) == -1);

    // Reports: nothing until features are selected, then 0x09 every 4 ms.
    switch2_pro_reset();
    now_us = 10000;
    switch2_pro_task();
    assert(report_count == 0);
    // Commands arrive over bulk; this one spans two reads of one packet each.
    auto init = bytes({ 0x03, 0x91, 0x00, 0x0d, 0x00, 0x08, 0, 0 }, { 0x01, 0x00, 0x31, 0x7e, 0xc6, 0xeb, 0xf1, 0x48 });
    auto mask = bytes({ 0x0c, 0x91, 0x00, 0x02, 0x00, 0x04, 0, 0 }, { 0x07, 0, 0, 0 });
    vendor_in = init;
    vendor_in.insert(vendor_in.end(), mask.begin(), mask.end());
    for (int i = 0; i < 4; i++) switch2_pro_task();
    assert(vendor_out.size() == 12 + 12 && vendor_out[3] == 0x0d && vendor_out[12 + 3] == 0x02);
    assert(report_count == 1 && last_id == 0x09);
    switch2_pro_task();
    assert(report_count == 1);  // Before the next 4 ms slot
    now_us += 4000;

    // A (HORI button 3), back L4, left stick right.
    const uint8_t horipad[8] = { 0x04, 0, 0x0F, 0xFF, 0x80, 0x80, 0x80, 0 };
    switch_pro_update_horipad(horipad, sizeof(horipad));
    switch_pro_raw_input(0x0009000F, 1);
    switch2_pro_task();
    assert(report_count == 2 && last_report[0] == 1);  // Counter
    assert(last_report[2] == 0x02);  // A
    assert(last_report[4] == 0x08);  // GL
    switch_pro_raw_input(0x00090012, 1);  // R5 is R3
    switch_pro_raw_input(0x00090011, 1);  // L5 is L3
    now_us += 4000;
    switch2_pro_task();
    assert((last_report[2] & 0x80) && (last_report[3] & 0x80));
    switch_pro_raw_input(0x00090012, 0);
    switch_pro_raw_input(0x00090011, 0);
    uint16_t lx = last_report[5] | ((last_report[6] & 0x0F) << 8);
    assert(lx == 4080);

    // Report 0x05 when the console selects it: report 0x30 button layout.
    command(bytes({ 0x03, 0x91, 0x00, 0x0a, 0x00, 0x04, 0, 0 }, { 0x05, 0, 0, 0 }));
    now_us += 4000;
    switch2_pro_task();
    assert(last_id == 0x05 && last_report[4] == 0x08 && last_report[7] == 0x02);  // A, GL
    assert(last_report[0x1f] == 0xa5 && last_report[0x20] == 0x0e);
}
