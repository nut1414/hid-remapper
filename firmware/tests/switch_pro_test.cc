#include <cassert>
#include <cstdint>
#include <cmath>
#include <cstring>

#include "crc.h"
#include "switch_pro.h"

static uint64_t now_us = 0;
static uint8_t last_id = 0;
static uint8_t last_payload[63];
static uint16_t last_len = 0;
static int report_count = 0;
static uint8_t flash_cal[SWITCH_PRO_CAL_IMAGE_SIZE];
static int flash_writes = 0;

uint64_t get_time() { return now_us; }
uint64_t get_unique_id() { return 0x123456789ABCDEF0ULL; }
void do_persist_switch_pro_cal(const uint8_t* buffer) {
    memcpy(flash_cal, buffer, sizeof(flash_cal));
    flash_writes++;
}
const uint8_t* get_persisted_switch_pro_cal() { return flash_cal; }
bool tud_hid_n_ready(uint8_t) { return true; }
bool tud_hid_n_report(uint8_t, uint8_t id, const void* payload, uint16_t len) {
    last_id = id;
    last_len = len;
    report_count++;
    memcpy(last_payload, payload, len);
    return true;
}

static int16_t sample_at(uint8_t offset) {
    return int16_t(uint16_t(last_payload[offset]) | (uint16_t(last_payload[offset + 1]) << 8));
}

static void read_user_cal(uint8_t* out) {
    uint8_t spi_read[15] = {};
    spi_read[9] = 0x10;
    spi_read[10] = 0x26;
    spi_read[11] = 0x80;
    spi_read[14] = 4;
    switch_pro_handle_set_report(0x01, spi_read, sizeof(spi_read));
    switch_pro_task();
    assert(last_id == 0x21 && last_payload[12] == 0x90 && last_payload[18] == 4);
    memcpy(out, last_payload + 19, 4);
}

int main() {
    // Calibration saved by an earlier session is loaded at the first reset.
    const uint32_t magic = 0x31435053;
    memset(flash_cal, 0xFF, sizeof(flash_cal));
    memcpy(flash_cal, &magic, 4);
    flash_cal[4 + 0x26] = 0xB2;
    flash_cal[4 + 0x27] = 0xA1;
    uint32_t crc = crc32(flash_cal, 4 + 256);
    memcpy(flash_cal + 4 + 256, &crc, 4);
    switch_pro_reset();
    uint8_t cal[4];
    read_user_cal(cal);
    assert(cal[0] == 0xB2 && cal[1] == 0xA1 && cal[2] == 0xFF);

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
    switch_pro_raw_input(0x00200453, 1000);
    switch_pro_raw_input(0x00200454, 2000);
    switch_pro_raw_input(0x00200455, 16384);
    switch_pro_raw_input(0x00200457, 100);
    switch_pro_raw_input(0x00200458, 200);
    switch_pro_raw_input(0x00200459, 300);
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

    // A button change pre-empts the 4 ms stream after one USB frame and
    // restarts the stream from that report.
    switch_pro_report_complete(0x30);
    switch_pro_latency_stats_t stats;
    switch_pro_get_latency_stats(&stats);
    int sent = report_count;
    now_us = 4400;
    switch_pro_input_received();
    gamepad[0] = 0x02;  // B
    switch_pro_update_horipad(gamepad, sizeof(gamepad));
    now_us = 4500;
    switch_pro_task();
    assert(report_count == sent);  // Too soon after the last report
    now_us = 5000;
    switch_pro_task();
    assert(report_count == sent + 1 && last_id == 0x30 && (last_payload[2] & 4));
    // The early report covers 1 ms of a 4 ms slot, so its gyro is scaled to a
    // quarter; acceleration is not scaled.
    assert(sample_at(12) == 500 && sample_at(16) == 4096);
    assert(sample_at(18) == 40 && sample_at(20) == -22 && sample_at(22) == 67);
    now_us = 5300;
    switch_pro_report_complete(0x30);
    now_us = 8000;
    switch_pro_task();
    assert(report_count == sent + 1);  // No change, no early report
    now_us = 9000;
    switch_pro_task();
    assert(report_count == sent + 2);  // Stream resumes 4 ms after the early report
    switch_pro_get_latency_stats(&stats);
    assert(stats.inputs == 1 && stats.reports == 2 && stats.early_reports == 1);
    assert(stats.button_count == 1 && stats.button_sum_us == 900 && stats.button_max_us == 900);
    assert(stats.input_count == 1 && stats.input_sum_us == 900);

    // Calibration writes are saved once the console has been quiet for 250 ms,
    // and only when they change something.
    uint8_t spi_write[19] = {};
    spi_write[9] = 0x11;
    spi_write[10] = 0x28;
    spi_write[11] = 0x80;
    spi_write[14] = 4;
    spi_write[15] = 0x12;
    spi_write[16] = 0x34;
    spi_write[17] = 0x56;
    spi_write[18] = 0x78;
    now_us = 100000;
    switch_pro_handle_set_report(0x01, spi_write, sizeof(spi_write));
    switch_pro_task();
    assert(last_id == 0x21 && flash_writes == 0);
    now_us = 300000;
    switch_pro_task();
    assert(flash_writes == 0);
    now_us = 350000;
    switch_pro_task();
    assert(flash_writes == 1);
    assert(flash_cal[4 + 0x28] == 0x12 && flash_cal[4 + 0x2B] == 0x78 && flash_cal[4 + 0x26] == 0xB2);
    crc = crc32(flash_cal, 4 + 256);
    assert(!memcmp(&crc, flash_cal + 4 + 256, 4));
    switch_pro_handle_set_report(0x01, spi_write, sizeof(spi_write));
    now_us = 700000;
    switch_pro_task();
    switch_pro_task();
    assert(flash_writes == 1);  // Same data, no flash write

    // Touchpads. The IMU sample is stale by now, so gyro carries only pad aim.
    auto pads = [](int32_t lx, int32_t ly, int32_t lp, int32_t rx, int32_t ry, int32_t rp) {
        const int32_t values[6] = { lx, ly, lp, rx, ry, rp };
        for (uint32_t i = 0; i < 6; i++) switch_pro_raw_input(0xFFFB0001 + i, values[i]);
        switch_pro_input_decoded();
    };
    auto stream = [&]() {
        now_us += 4000;
        int before = report_count;
        switch_pro_task();
        assert(report_count == before + 1 && last_id == 0x30);
    };
    const uint8_t neutral[8] = { 0, 0, 0x0F, 0x80, 0x80, 0x80, 0x80, 0 };
    switch_pro_update_horipad(neutral, sizeof(neutral));
    now_us = 800000;
    switch_pro_task();
    pads(0, 0, 0, 1000, 1000, 500);
    stream();
    assert(sample_at(20) == 0 && sample_at(22) == 0);  // First touch does not jump
    pads(0, 0, 0, 1100, 1050, 500);
    stream();
    assert(sample_at(20) == -50 && sample_at(22) == -100);  // Pitch, yaw
    assert(sample_at(32) == -50 && sample_at(46) == -100);  // All 3 samples
    // Gravity (stale IMU: 4096 on Z) turns with the added pitch.
    const float pitch = -50 * 3 * 0.005f * (936.0f / 13371.0f) * (3.14159265f / 180);
    assert(sample_at(12) == lroundf(-sinf(pitch) * 4096));
    assert(sample_at(16) == lroundf(cosf(pitch) * 4096));
    pads(0, 0, 0, 51100, 1050, 500);  // A flick beyond one report's range
    stream();
    assert(sample_at(22) == -32768);
    stream();
    assert(sample_at(22) == -(50000 - 32768));  // Remainder in the next report
    stream();
    assert(sample_at(22) == 0);

    // Right pad press is ZR, sent early; it releases below the lower threshold.
    pads(0, 0, 0, 11100, 1050, 4000);
    now_us += 1000;
    int sent_before = report_count;
    switch_pro_task();
    assert(report_count == sent_before + 1 && (last_payload[2] & 0x80));
    pads(0, 0, 0, 11100, 1050, 3000);
    stream();
    assert(last_payload[2] & 0x80);
    pads(0, 0, 0, 11100, 1050, 2500);
    now_us += 1000;
    switch_pro_task();
    assert(!(last_payload[2] & 0x80));

    // Left pad press at an edge is the d-pad; touch alone or a centre press is not.
    pads(0, 20000, 500, 0, 0, 0);
    stream();
    assert(!(last_payload[4] & 0x0F));
    pads(0, 20000, 5000, 0, 0, 0);
    now_us += 1000;
    switch_pro_task();
    assert((last_payload[4] & 0x0F) == 0x02);  // Up
    pads(-20000, -20000, 5000, 0, 0, 0);
    now_us += 1000;
    switch_pro_task();
    assert((last_payload[4] & 0x0F) == 0x09);  // Down + left
    pads(100, 100, 5000, 0, 0, 0);
    now_us += 1000;
    switch_pro_task();
    assert(!(last_payload[4] & 0x0F));

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
