/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 Jacek Fedorynski
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <tusb.h>

#include "config.h"
#include "globals.h"
#include "our_descriptor.h"
#include "platform.h"
#include "remapper.h"
#include "switch2_pro.h"
#include "switch_pro.h"

// These IDs are bogus. If you want to distribute any hardware using this,
// you will have to get real ones.
#define USB_VID 0xCAFE
#define USB_PID 0xBAF2

tusb_desc_device_t desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,

    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,

    .bNumConfigurations = 0x01,
};

const uint8_t configuration_descriptor0[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN + TUD_HID_DESC_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_DESCRIPTOR(0, 0, HID_ITF_PROTOCOL_KEYBOARD, our_descriptors[0].descriptor_length, 0x81, CFG_TUD_HID_EP_BUFSIZE, 1),
    TUD_HID_DESCRIPTOR(1, 0, HID_ITF_PROTOCOL_NONE, config_report_descriptor_length, 0x83, CFG_TUD_HID_EP_BUFSIZE, 1),
};

const uint8_t configuration_descriptor1[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN + TUD_HID_DESC_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_DESCRIPTOR(0, 0, HID_ITF_PROTOCOL_KEYBOARD, our_descriptors[1].descriptor_length, 0x81, CFG_TUD_HID_EP_BUFSIZE, 1),
    TUD_HID_DESCRIPTOR(1, 0, HID_ITF_PROTOCOL_NONE, config_report_descriptor_length, 0x83, CFG_TUD_HID_EP_BUFSIZE, 1),
};

const uint8_t configuration_descriptor2[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN + TUD_HID_DESC_LEN, 0, 100),
    TUD_HID_INOUT_DESCRIPTOR(0, 0, HID_ITF_PROTOCOL_NONE, our_descriptors[2].descriptor_length, 0x02, 0x81, CFG_TUD_HID_EP_BUFSIZE, 1),
    TUD_HID_DESCRIPTOR(1, 0, HID_ITF_PROTOCOL_NONE, config_report_descriptor_length, 0x83, CFG_TUD_HID_EP_BUFSIZE, 1),
};

const uint8_t configuration_descriptor3[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN + TUD_HID_DESC_LEN, 0, 100),
    TUD_HID_DESCRIPTOR(0, 0, HID_ITF_PROTOCOL_NONE, our_descriptors[3].descriptor_length, 0x81, CFG_TUD_HID_EP_BUFSIZE, 1),
    TUD_HID_DESCRIPTOR(1, 0, HID_ITF_PROTOCOL_NONE, config_report_descriptor_length, 0x83, CFG_TUD_HID_EP_BUFSIZE, 1),
};

const uint8_t configuration_descriptor4[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN + TUD_HID_DESC_LEN, 0, 100),
    TUD_HID_INOUT_DESCRIPTOR(0, 0, HID_ITF_PROTOCOL_NONE, our_descriptors[4].descriptor_length, 0x02, 0x81, CFG_TUD_HID_EP_BUFSIZE, 1),
    TUD_HID_DESCRIPTOR(1, 0, HID_ITF_PROTOCOL_NONE, config_report_descriptor_length, 0x83, CFG_TUD_HID_EP_BUFSIZE, 1),
};

const uint8_t configuration_descriptor5[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN + TUD_HID_DESC_LEN, 0, 100),
    TUD_HID_DESCRIPTOR(0, 0, HID_ITF_PROTOCOL_NONE, our_descriptors[5].descriptor_length, 0x81, CFG_TUD_HID_EP_BUFSIZE, 1),
    TUD_HID_DESCRIPTOR(1, 0, HID_ITF_PROTOCOL_NONE, config_report_descriptor_length, 0x83, CFG_TUD_HID_EP_BUFSIZE, 1),
};

const uint8_t configuration_descriptor6[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN + TUD_HID_DESC_LEN, 0, 100),
    TUD_HID_INOUT_DESCRIPTOR(0, 0, HID_ITF_PROTOCOL_NONE, switch_pro_report_descriptor_length, 0x02, 0x81, CFG_TUD_HID_EP_BUFSIZE, 1),
    TUD_HID_DESCRIPTOR(1, 0, HID_ITF_PROTOCOL_NONE, config_report_descriptor_length, 0x83, CFG_TUD_HID_EP_BUFSIZE, 1),
};

// Switch 2 Pro Controller: HID and vendor (command) functions, each behind an
// interface association as on the real controller, then our config interface.
// The real controller's headset audio functions are left out.
const uint8_t configuration_descriptor7[] = {
    TUD_CONFIG_DESCRIPTOR(1, 3, 0, TUD_CONFIG_DESC_LEN + 8 + TUD_HID_INOUT_DESC_LEN + 8 + TUD_VENDOR_DESC_LEN + TUD_HID_DESC_LEN, 0xC0, 500),
    8, TUSB_DESC_INTERFACE_ASSOCIATION, 0, 1, TUSB_CLASS_HID, 0, 0, 0,
    TUD_HID_INOUT_DESCRIPTOR(0, 0, HID_ITF_PROTOCOL_NONE, switch2_pro_report_descriptor_length, 0x01, 0x81, CFG_TUD_HID_EP_BUFSIZE, 4),
    8, TUSB_DESC_INTERFACE_ASSOCIATION, 1, 1, TUSB_CLASS_VENDOR_SPECIFIC, 0, 0, 0,
    TUD_VENDOR_DESCRIPTOR(1, 0, 0x02, 0x82, 64),
    TUD_HID_DESCRIPTOR(2, 0, HID_ITF_PROTOCOL_NONE, config_report_descriptor_length, 0x83, CFG_TUD_HID_EP_BUFSIZE, 1),
};

const uint8_t* configuration_descriptors[] = {
    configuration_descriptor0,
    configuration_descriptor1,
    configuration_descriptor2,
    configuration_descriptor3,
    configuration_descriptor4,
    configuration_descriptor5,
    configuration_descriptor6,
    configuration_descriptor7,
};

char const* string_desc_arr[] = {
    (const char[]){ 0x09, 0x04 },  // 0: is supported language is English (0x0409)
#ifdef PICO_RP2350
    "RP2350",  // 1: Manufacturer
#else
    "RP2040",  // 1: Manufacturer
#endif
    "HID Remapper XXXX",  // 2: Product
    "123456789012",       // 3: Serial Number
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const* tud_descriptor_device_cb() {
    if ((our_descriptor->vid != 0) && (our_descriptor->pid != 0)) {
        desc_device.idVendor = our_descriptor->vid;
        desc_device.idProduct = our_descriptor->pid;
    }
    if (our_descriptor_number == SWITCH2_PRO_DESCRIPTOR_INDEX) {
        // Interface associations, as the real controller declares.
        desc_device.bDeviceClass = TUSB_CLASS_MISC;
        desc_device.bDeviceSubClass = MISC_SUBCLASS_COMMON;
        desc_device.bDeviceProtocol = MISC_PROTOCOL_IAD;
        desc_device.bcdDevice = 0x0200;
    }
    return (uint8_t const*) &desc_device;
}

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    return configuration_descriptors[our_descriptor->idx];
}

// Invoked when received GET HID REPORT DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const* tud_hid_descriptor_report_cb(uint8_t itf) {
    if (itf == 0) {
        return our_descriptor->usb_descriptor != nullptr ? our_descriptor->usb_descriptor : our_descriptor->descriptor;
    } else if (itf == 1) {
        return config_report_descriptor;
    }

    return NULL;
}

static uint16_t _desc_str[32];

const char id_chars[33] = "0123456789ABCDEFGHIJKLMNOPQRSTUV";

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
        // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

        if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0])))
            return NULL;

        const char* str = string_desc_arr[index];
        if (our_descriptor_number == SWITCH_PRO_DESCRIPTOR_INDEX) {
            if (index == 1) str = "Nintendo Co., Ltd.";
            if (index == 2) str = "Pro Controller";
        }
        if (our_descriptor_number == SWITCH2_PRO_DESCRIPTOR_INDEX) {
            if (index == 1) str = "Nintendo";
            if (index == 2) str = "Switch 2 Pro Controller";
        }

        // Cap at max char
        chr_count = strlen(str);
        if (chr_count > 31)
            chr_count = 31;

        // Convert ASCII string into UTF-16
        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }

        if (index == 2 && !is_switch_pro_descriptor(our_descriptor_number)) {
            uint64_t unique_id = get_unique_id();
            for (uint8_t i = 0; i < 4; i++) {
                _desc_str[1 + chr_count - 4 + i] = id_chars[(unique_id >> (15 - i * 5)) & 0x1F];
            }
        }

        if (index == 3) {
            uint64_t unique_id = get_unique_id();
            for (uint8_t i = 0; i < 12; i++) {
                _desc_str[1 + i] = id_chars[(unique_id >> (55 - i * 5)) & 0x1F];
            }
        }
    }

    // first byte is length (including header), second byte is string type
    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);

    return _desc_str;
}

uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
    if (itf == 0) {
        return handle_get_report0(report_id, buffer, reqlen);
    } else {
        return handle_get_report1(report_id, buffer, reqlen);
    }
}

void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
    if (itf == 0) {
        if ((report_id == 0) && (report_type == 0) && (bufsize > 0)) {
            report_id = buffer[0];
            buffer++;
            bufsize--;
        }
        handle_set_report0(report_id, buffer, bufsize);
    } else {
        handle_set_report1(report_id, buffer, bufsize);
    }
}

void tud_hid_set_protocol_cb(uint8_t instance, uint8_t protocol) {
    printf("tud_hid_set_protocol_cb %d %d\n", instance, protocol);
    boot_protocol_keyboard = (protocol == HID_PROTOCOL_BOOT);
    boot_protocol_updated = true;
}

void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len) {
    if (instance == 0 && len > 0 && our_descriptor_number == SWITCH_PRO_DESCRIPTOR_INDEX) {
        switch_pro_report_complete(report[0]);
    }
}

// The Switch 2 asks for device info and factory data with vendor requests.
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request) {
    static uint8_t data[SWITCH2_PRO_FACTORY_DATA_LEN];
    if (our_descriptor_number != SWITCH2_PRO_DESCRIPTOR_INDEX) {
        return false;
    }
    if (stage != CONTROL_STAGE_SETUP) {
        return true;
    }
    if (request->bmRequestType_bit.direction == TUSB_DIR_IN) {
        int32_t len = switch2_pro_vendor_request(request->bRequest, data);
        if (len < 0) {
            return false;
        }
        return tud_control_xfer(rhport, request, data, len < request->wLength ? len : request->wLength);
    }
    if (request->wLength > 0) {
        // Nothing needs the data; take it so the transfer completes.
        return tud_control_xfer(rhport, request, data, request->wLength < sizeof(data) ? request->wLength : sizeof(data));
    }
    return tud_control_status(rhport, request);
}

void tud_mount_cb() {
    if (is_switch_pro_descriptor(our_descriptor_number)) {
        switch_pro_reset();
    }
    if (our_descriptor_number == SWITCH2_PRO_DESCRIPTOR_INDEX) {
        switch2_pro_reset();
    }
    reset_resolution_multiplier();
    if (boot_protocol_keyboard) {
        boot_protocol_keyboard = false;
        boot_protocol_updated = true;
    }
}

void tud_suspend_cb(bool remote_wakeup_en) {
    printf("tud_suspend_cb\n");
}

void tud_resume_cb() {
    printf("tud_resume_cb\n");
}
