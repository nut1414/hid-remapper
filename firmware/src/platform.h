#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#include <stdint.h>
#include <types.h>

void do_persist_config(uint8_t* buffer);
// Switch Pro user calibration image, SWITCH_PRO_CAL_IMAGE_SIZE bytes.
void do_persist_switch_pro_cal(const uint8_t* buffer);
const uint8_t* get_persisted_switch_pro_cal();

void reset_to_bootloader();
void pair_new_device();
void clear_bonds();
void flash_b_side();

void my_mutexes_init();
void my_mutex_enter(MutexId id);
void my_mutex_exit(MutexId id);

uint64_t get_time();
uint64_t get_unique_id();

uint32_t get_gpio_valid_pins_mask();
void set_gpio_inout_masks(uint32_t in_mask, uint32_t out_mask);

#endif
