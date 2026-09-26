#include <cmath>
#include <cstring>

#include "switch2_motion.h"

namespace {

constexpr float PI = 3.14159265f;
constexpr float GYRO_RAD_PER_UNIT = (2000.0f / 32768) * (PI / 180);
constexpr float ACCEL_UNITS_PER_G = 16384;
// Gravity pulls tilt back to the truth this fast (1/s); ignored while the
// controller is being shaken.
constexpr float GRAVITY_GAIN = 1.0f;
constexpr float GRAVITY_TOLERANCE = 0.2f;
// Touchpad: about 70 degrees for a full swipe across the pad.
constexpr float PAD_RAD_PER_UNIT = 1.8e-5f;
constexpr float PAD_PITCH_LIMIT = 1.4f;  // About 80 degrees

// Resting flat: body Z up is world Y up, body Y forward is world -Z.
const float rest[4] = { 0.70710678f, -0.70710678f, 0, 0 };

float real[4];      // Real controller orientation
float pad_yaw;      // Radians, about world up; positive turns left
float pad_pitch;    // Radians, about body right; positive looks up
float rate_dps[3];  // Last body rates, pad included

void multiply(const float* a, const float* b, float* out) {
    float r[4] = {
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    };
    memcpy(out, r, sizeof(r));
}

void normalize(float* q) {
    float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (uint8_t i = 0; i < 4; i++) q[i] /= n;
}

// World up (Y) seen in the body frame: the second row of the rotation.
void body_up(const float* q, float* up) {
    up[0] = 2 * (q[1] * q[2] + q[0] * q[3]);
    up[1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
    up[2] = 2 * (q[2] * q[3] - q[0] * q[1]);
}

void output(float* q) {
    const float yaw[4] = { cosf(pad_yaw / 2), 0, sinf(pad_yaw / 2), 0 };
    const float pitch[4] = { cosf(pad_pitch / 2), sinf(pad_pitch / 2), 0, 0 };
    multiply(yaw, real, q);
    multiply(q, pitch, q);
    normalize(q);
}

void put_bits(uint8_t* buf, uint16_t bitpos, uint32_t value, uint8_t bits) {
    for (uint8_t i = 0; i < bits; i++, bitpos++) {
        if (value & (1u << i)) buf[bitpos / 8] |= 1 << (bitpos % 8);
    }
}

}  // namespace

void switch2_motion_reset() {
    memcpy(real, rest, sizeof(real));
    pad_yaw = pad_pitch = 0;
    memset(rate_dps, 0, sizeof(rate_dps));
}

void switch2_motion_update(const int16_t accel[3], const int16_t gyro[3], bool motion,
                           int32_t pad_x, int32_t pad_y, float dt) {
    float w[3] = {};
    if (motion) {
        for (uint8_t i = 0; i < 3; i++) w[i] = gyro[i] * GYRO_RAD_PER_UNIT;
        float a[3] = { accel[0] / ACCEL_UNITS_PER_G, accel[1] / ACCEL_UNITS_PER_G, accel[2] / ACCEL_UNITS_PER_G };
        float norm = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        float correction[3] = {};
        if (fabsf(norm - 1) < GRAVITY_TOLERANCE) {
            float up[3];
            body_up(real, up);
            // Turn towards the measured gravity (Mahony complementary filter).
            correction[0] = (a[1] * up[2] - a[2] * up[1]) / norm * GRAVITY_GAIN;
            correction[1] = (a[2] * up[0] - a[0] * up[2]) / norm * GRAVITY_GAIN;
            correction[2] = (a[0] * up[1] - a[1] * up[0]) / norm * GRAVITY_GAIN;
        }
        const float half[4] = {
            0,
            (w[0] + correction[0]) * dt / 2,
            (w[1] + correction[1]) * dt / 2,
            (w[2] + correction[2]) * dt / 2,
        };
        float delta[4];
        multiply(real, half, delta);
        for (uint8_t i = 0; i < 4; i++) real[i] += delta[i];
        normalize(real);
    }

    // Right on the pad turns right, up looks up.
    float yaw_step = -pad_x * PAD_RAD_PER_UNIT;
    float old_pitch = pad_pitch;
    pad_yaw = remainderf(pad_yaw + yaw_step, 2 * PI);
    pad_pitch += pad_y * PAD_RAD_PER_UNIT;
    if (pad_pitch > PAD_PITCH_LIMIT) pad_pitch = PAD_PITCH_LIMIT;
    if (pad_pitch < -PAD_PITCH_LIMIT) pad_pitch = -PAD_PITCH_LIMIT;

    // Body rates: the real gyro, plus pad yaw about the up axis and pad
    // pitch about the right axis.
    float q[4];
    output(q);
    float up[3];
    body_up(q, up);
    for (uint8_t i = 0; i < 3; i++) {
        float rate = w[i];
        if (dt > 0) rate += up[i] * yaw_step / dt + (i == 0 ? (pad_pitch - old_pitch) / dt : 0);
        rate_dps[i] = rate * 180 / PI;
    }
}

void switch2_motion_get(switch2_motion_t* out) {
    output(out->q);
    body_up(out->q, out->accel);
    memcpy(out->gyro_dps, rate_dps, sizeof(rate_dps));
}

void switch2_motion_pack_mode12(const switch2_motion_t& motion, uint8_t* out) {
    memset(out, 0, 26);
    uint8_t omitted = 0;
    for (uint8_t i = 1; i < 4; i++) {
        if (fabsf(motion.q[i]) > fabsf(motion.q[omitted])) omitted = i;
    }
    put_bits(out, 0, omitted, 2);
    // The three kept components follow the dropped one, in order, each as
    // the top 26 bits of (value / largest) mapped from [-1, 1] to 32 bits.
    const uint16_t positions[3] = { 8, 39, 70 };
    for (uint8_t n = 0; n < 3; n++) {
        float v = motion.q[(omitted + 1 + n) % 4] / motion.q[omitted];
        double scaled = (v * 0.5 + 0.5) * 4294967295.0;
        uint32_t encoded = scaled <= 0 ? 0 : scaled >= 4294967295.0 ? 0xFFFFFFFF : uint32_t(scaled);
        put_bits(out, positions[n], encoded >> 6, 26);
    }
    for (uint8_t i = 0; i < 3; i++) {
        int32_t g = int32_t(motion.accel[i] * 268435456.0f);  // g * 2^28
        memcpy(out + 12 + i * 4, &g, 4);
    }
}
