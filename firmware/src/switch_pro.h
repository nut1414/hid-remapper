#ifndef _SWITCH_PRO_H_
#define _SWITCH_PRO_H_

#include <stdint.h>

// Persisted user calibration: magic, the 0x8000-0x80FF SPI mirror, CRC32,
// padded to whole flash pages.
constexpr uint32_t SWITCH_PRO_CAL_IMAGE_SIZE = 512;

extern const uint8_t switch_pro_report_descriptor[];
constexpr uint32_t switch_pro_report_descriptor_length = 120;

// Returned by ConfigCommand::GET_LATENCY_STATS; counters reset on each read.
struct switch_pro_latency_stats_t {
    uint32_t inputs;           // host input reports received
    uint32_t reports;          // 0x30 reports queued
    uint16_t early_reports;    // 0x30 reports sent ahead of the stream interval
    uint16_t input_gap_max_us;
    uint16_t input_count;      // 0x30 reports read that carried new input
    uint16_t input_max_us;
    uint32_t input_sum_us;     // oldest unsent input arrival -> 0x30 read
    uint16_t button_count;     // 0x30 reports read that carried a button change
    uint16_t button_max_us;
    uint32_t button_sum_us;    // button change arrival -> 0x30 read
};

static_assert(sizeof(switch_pro_latency_stats_t) == 28, "Stats must fit a config reply");

void switch_pro_reset();
void switch_pro_update_horipad(const uint8_t* report, uint16_t len);
void switch_pro_imu_input(uint32_t usage, int32_t value);
void switch_pro_handle_set_report(uint8_t report_id, const uint8_t* buffer, uint16_t len);
void switch_pro_task();
void switch_pro_input_received();
void switch_pro_report_complete(uint8_t report_id);
void switch_pro_get_latency_stats(switch_pro_latency_stats_t* out);

#endif
