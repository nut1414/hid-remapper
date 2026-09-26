#ifndef _SWITCH2_PRO_H_
#define _SWITCH2_PRO_H_

#include <stdint.h>

// Switch 2 Pro Controller (057E:2069) emulation, written from the protocol
// notes at https://github.com/ndeadly/switch2_controller_research. The
// console sends commands over a vendor bulk interface and reads input
// reports 0x05 or 0x09 from the HID interface.

extern const uint8_t switch2_pro_report_descriptor[];
constexpr uint32_t switch2_pro_report_descriptor_length = 97;

constexpr uint8_t SWITCH2_PRO_REPORT_LEN = 63;
constexpr uint16_t SWITCH2_PRO_MAX_REPLY = 8 + 8 + 80;
constexpr uint8_t SWITCH2_PRO_DEVICE_INFO_LEN = 18;
constexpr uint8_t SWITCH2_PRO_FACTORY_DATA_LEN = 64;

void switch2_pro_reset();
void switch2_pro_task();

// Handles one complete command; returns the reply length (0 = no reply).
uint16_t switch2_pro_command(const uint8_t* request, uint16_t len, uint8_t* reply);
// Fills the reply to vendor control request 2 (device info) or 3 (factory
// data); returns its length, or -1 for requests it does not know.
int32_t switch2_pro_vendor_request(uint8_t request, uint8_t* data);
// Builds the currently selected input report; returns its ID.
uint8_t switch2_pro_build_report(uint8_t* out);
void switch2_pro_handle_set_report(uint8_t report_id, const uint8_t* buffer, uint16_t len);

#endif
