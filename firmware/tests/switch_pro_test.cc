#include <cassert>
#include <cstdint>
#include <cstring>

#include "switch_pro.h"

static uint64_t now_us = 0;
static uint8_t last_id = 0;
static uint8_t last_payload[63];
static uint16_t last_len = 0;

uint64_t get_time() { return now_us; }
uint64_t get_unique_id() { return 0x123456789ABCDEF0ULL; }
bool tud_hid_n_ready(uint8_t) { return true; }
bool tud_hid_n_report(uint8_t, uint8_t id, const void* payload, uint16_t len) {
    last_id = id;
    last_len = len;
    memcpy(last_payload, payload, len);
    return true;
}

static int16_t sample_at(uint8_t offset) {
    return int16_t(uint16_t(last_payload[offset]) | (uint16_t(last_payload[offset + 1]) << 8));
}

int main() {
    switch_pro_reset();
    const uint8_t handshake[] = { 0x01 };
    switch_pro_handle_set_report(0x80, handshake, sizeof(handshake));
    switch_pro_task();
    assert(last_id == 0x81 && last_len == 63);
    assert(last_payload[0] == 0x01 && last_payload[2] == 0x03);

    uint8_t command[11] = {};
    command[9] = 0x03;
    command[10] = 0x30;
    switch_pro_handle_set_report(0x01, command, sizeof(command));
    switch_pro_task();
    assert(last_id == 0x21 && last_payload[12] == 0x80 && last_payload[13] == 0x03);

    command[9] = 0x40;
    command[10] = 0x01;
    switch_pro_handle_set_report(0x01, command, sizeof(command));
    switch_pro_task();
    assert(last_id == 0x21 && last_payload[13] == 0x40);

    uint8_t gamepad[8] = { 0x01, 0, 0x00, 0x80, 0x80, 0x80, 0x80, 0 };
    switch_pro_update_horipad(gamepad, sizeof(gamepad));
    now_us = 1000;
    switch_pro_imu_input(0x00200453, 1000);
    switch_pro_imu_input(0x00200454, 2000);
    switch_pro_imu_input(0x00200455, 16384);
    switch_pro_imu_input(0x00200457, 100);
    switch_pro_imu_input(0x00200458, 200);
    switch_pro_imu_input(0x00200459, 300);
    now_us = 4000;
    switch_pro_task();
    assert(last_id == 0x30 && last_len == 63);
    assert((last_payload[2] & 1) && (last_payload[4] & 2));  // Y, D-pad up
    for (uint8_t n = 0; n < 3; n++) {
        uint8_t base = 12 + n * 12;
        assert(sample_at(base) == 500);
        assert(sample_at(base + 2) == -250);
        assert(sample_at(base + 4) == 4096);
        assert(sample_at(base + 6) == 160);
        assert(sample_at(base + 8) == -90);
        assert(sample_at(base + 10) == 270);
    }

    uint8_t spi_read[15] = {};
    spi_read[9] = 0x10;
    spi_read[10] = 0x20;
    spi_read[11] = 0x60;
    spi_read[14] = 24;
    switch_pro_handle_set_report(0x01, spi_read, sizeof(spi_read));
    switch_pro_task();
    assert(last_id == 0x21 && last_payload[12] == 0x90);
    assert(last_payload[18] == 24 && last_payload[26] == 0x40);
}
