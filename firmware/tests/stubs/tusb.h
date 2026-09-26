#pragma once
#include <stdint.h>

bool tud_hid_n_ready(uint8_t instance);
bool tud_hid_n_report(uint8_t instance, uint8_t report_id, const void* report, uint16_t len);

uint32_t tud_vendor_n_available(uint8_t itf);
uint32_t tud_vendor_n_read(uint8_t itf, void* buffer, uint32_t bufsize);
uint32_t tud_vendor_n_write(uint8_t itf, void const* buffer, uint32_t bufsize);
uint32_t tud_vendor_n_write_flush(uint8_t itf);
uint32_t tud_vendor_n_write_available(uint8_t itf);
