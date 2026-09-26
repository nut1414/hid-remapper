#pragma once
#include <stdint.h>

bool tud_hid_n_ready(uint8_t instance);
bool tud_hid_n_report(uint8_t instance, uint8_t report_id, const void* report, uint16_t len);
