#ifndef _SWITCH2_MOTION_H_
#define _SWITCH2_MOTION_H_

#include <stdint.h>

// Orientation for the Switch 2 Pro Controller profile. The real gyro is
// integrated (tilt kept honest by gravity), then the right touchpad adds
// yaw about the world's vertical and pitch about the controller's right
// axis, like a mouse: the console gets an orientation, and gravity to match,
// so pad aim stays where it is put.
//
// Body axes are X right, Y forward, Z up; the world is Y up. Quaternions
// are (w, x, y, z) and rotate body vectors into the world.

struct switch2_motion_t {
    float q[4];
    float accel[3];     // Gravity seen by the controller, in g
    float gyro_dps[3];  // Body rates including pad aim, in degrees per second
};

void switch2_motion_reset();
// accel and gyro in Steam Controller units, pad movement in pad units,
// dt in seconds since the previous update.
void switch2_motion_update(const int16_t accel[3], const int16_t gyro[3], bool motion,
                           int32_t pad_x, int32_t pad_y, float dt);
void switch2_motion_get(switch2_motion_t* out);

// Packs an orientation as report 0x09 motion mode 12 (26 bytes): the
// largest component is dropped and the others scaled by it, then the
// accelerometer as g * 2^28.
void switch2_motion_pack_mode12(const switch2_motion_t& motion, uint8_t* out);

#endif
