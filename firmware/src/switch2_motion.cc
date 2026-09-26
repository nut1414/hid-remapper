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
// Touchpad: about 63 degrees for a full swipe across the pad.
constexpr float PAD_RAD_PER_UNIT = 1.62e-5f;
constexpr float PAD_PITCH_LIMIT = 1.4f;  // About 80 degrees
// Pad aim follows its target through a One Euro filter: slow movement is
// smoothed at MIN_CUTOFF (Hz), faster movement raises the cutoff by BETA
// per rad/s. BETA is kept low so flicks are smoothed too (a sudden flick
// lands over about 3 reports). DERIVATIVE_CUTOFF smooths the speed estimate.
constexpr float PAD_MIN_CUTOFF = 15;
constexpr float PAD_BETA = 2;
constexpr float PAD_DERIVATIVE_CUTOFF = 10;

// Resting flat: body Z up is world Y up, body Y forward is world -Z.
const float rest[4] = { 0.70710678f, -0.70710678f, 0, 0 };

float real[4];      // Real controller orientation
float real_rate[3];  // Last real gyro rates (rad/s)

// Pad aim in radians: yaw about world up (positive turns left), pitch about
// body right (positive looks up).
struct pad_axis_t {
    float target;  // Where the pad has put it
    float value;   // Smoothed, as sent
    float speed;   // Smoothed speed estimate (rad/s)
    float rate;    // Change over the last advance (rad/s)
};
pad_axis_t pad_yaw;
pad_axis_t pad_pitch;

float smoothing(float cutoff, float dt) {
    float tau = 1 / (2 * PI * cutoff);
    return 1 / (1 + tau / dt);
}

void follow(pad_axis_t& axis, float dt) {
    float speed = (axis.target - axis.value) / dt;
    axis.speed += smoothing(PAD_DERIVATIVE_CUTOFF, dt) * (speed - axis.speed);
    float cutoff = PAD_MIN_CUTOFF + PAD_BETA * fabsf(axis.speed);
    float step = smoothing(cutoff, dt) * (axis.target - axis.value);
    axis.value += step;
    axis.rate = step / dt;
}

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
    const float yaw[4] = { cosf(pad_yaw.value / 2), 0, sinf(pad_yaw.value / 2), 0 };
    const float pitch[4] = { cosf(pad_pitch.value / 2), sinf(pad_pitch.value / 2), 0, 0 };
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
    pad_yaw = pad_pitch = {};
    memset(real_rate, 0, sizeof(real_rate));
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

    memcpy(real_rate, w, sizeof(real_rate));

    // Right on the pad turns right, up looks up.
    pad_yaw.target -= pad_x * PAD_RAD_PER_UNIT;
    pad_pitch.target += pad_y * PAD_RAD_PER_UNIT;
    if (pad_pitch.target > PAD_PITCH_LIMIT) pad_pitch.target = PAD_PITCH_LIMIT;
    if (pad_pitch.target < -PAD_PITCH_LIMIT) pad_pitch.target = -PAD_PITCH_LIMIT;
    // Keep yaw small so float precision holds up; both move together.
    if (fabsf(pad_yaw.value) > 2 * PI) {
        float turns = 2 * PI * roundf(pad_yaw.value / (2 * PI));
        pad_yaw.value -= turns;
        pad_yaw.target -= turns;
    }
}

void switch2_motion_advance(float dt) {
    if (dt <= 0) return;
    follow(pad_yaw, dt);
    follow(pad_pitch, dt);
}

void switch2_motion_get(switch2_motion_t* out) {
    output(out->q);
    body_up(out->q, out->accel);
    // Body rates: the real gyro, plus pad yaw about the up axis and pad
    // pitch about the right axis.
    for (uint8_t i = 0; i < 3; i++) {
        float rate = real_rate[i] + out->accel[i] * pad_yaw.rate + (i == 0 ? pad_pitch.rate : 0);
        out->gyro_dps[i] = rate * 180 / PI;
    }
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
