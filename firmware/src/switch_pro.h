#ifndef _SWITCH_PRO_H_
#define _SWITCH_PRO_H_

#include <stdint.h>

extern const uint8_t switch_pro_report_descriptor[];
constexpr uint32_t switch_pro_report_descriptor_length = 120;

void switch_pro_reset();
void switch_pro_update_horipad(const uint8_t* report, uint16_t len);
void switch_pro_imu_input(uint32_t usage, int32_t value);
void switch_pro_handle_set_report(uint8_t report_id, const uint8_t* buffer, uint16_t len);
void switch_pro_task();

#endif
