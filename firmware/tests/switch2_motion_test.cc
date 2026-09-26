#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "switch2_motion.h"

static const int16_t flat_accel[3] = { 0, 0, 16384 };
static const int16_t no_gyro[3] = { 0, 0, 0 };

static const float PAD_RAD = 1.62e-5f;  // Must match switch2_motion.cc

static void settle() {
    for (int i = 0; i < 100; i++) switch2_motion_advance(0.004f);
}

static bool near(float a, float b, float tolerance) {
    return fabsf(a - b) < tolerance;
}

// Body forward (Y) in the world.
static void forward(const float* q, float* f) {
    f[0] = 2 * (q[1] * q[2] - q[0] * q[3]);
    f[1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
    f[2] = 2 * (q[2] * q[3] + q[0] * q[1]);
}

static uint32_t get_bits(const uint8_t* buf, int bitpos, int bits) {
    uint32_t value = 0;
    for (int i = 0; i < bits; i++, bitpos++) {
        if (buf[bitpos / 8] & (1 << (bitpos % 8))) value |= 1u << i;
    }
    return value;
}

// Independent decoder for mode 12, from the layout: 2-bit dropped index,
// then three 26-bit fields at bits 8, 39 and 70, then 3 x int32 accel.
static void unpack(const uint8_t* buf, float* q, float* accel) {
    int omitted = get_bits(buf, 0, 2);
    const int positions[3] = { 8, 39, 70 };
    float v[4];
    v[omitted] = 1;
    for (int n = 0; n < 3; n++) {
        double field = get_bits(buf, positions[n], 26) * 64.0;
        v[(omitted + 1 + n) % 4] = float(field / 4294967295.0 * 2 - 1);
    }
    float norm = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]);
    for (int i = 0; i < 4; i++) q[i] = v[i] / norm;
    for (int i = 0; i < 3; i++) {
        int32_t g;
        memcpy(&g, buf + 12 + i * 4, 4);
        accel[i] = g / 268435456.0f;
    }
}

static void check_roundtrip(const switch2_motion_t& motion) {
    uint8_t packed[26];
    switch2_motion_pack_mode12(motion, packed);
    float q[4];
    float accel[3];
    unpack(packed, q, accel);
    // q and -q are the same rotation.
    float dot = 0;
    for (int i = 0; i < 4; i++) dot += q[i] * motion.q[i];
    assert(fabsf(dot) > 0.99999f);
    for (int i = 0; i < 3; i++) assert(near(accel[i], motion.accel[i], 1e-6f));
}

int main() {
    switch2_motion_t m;

    // Resting flat: gravity straight up the body Z axis, facing world -Z.
    switch2_motion_reset();
    switch2_motion_get(&m);
    assert(near(m.accel[0], 0, 1e-5f) && near(m.accel[1], 0, 1e-5f) && near(m.accel[2], 1, 1e-5f));
    float f[3];
    forward(m.q, f);
    assert(near(f[2], -1, 1e-5f));
    check_roundtrip(m);

    // Pad right turns right (towards world +X) and leaves gravity alone.
    switch2_motion_update(flat_accel, no_gyro, true, 20000, 0, 0.004f);
    switch2_motion_advance(0.004f);
    switch2_motion_get(&m);
    assert(m.gyro_dps[2] < 0);  // Turning right about the up axis
    settle();
    switch2_motion_get(&m);
    forward(m.q, f);
    assert(near(asinf(f[0]), 20000 * PAD_RAD, 1e-3f) && near(f[1], 0, 1e-4f));
    assert(near(m.accel[2], 1, 1e-4f));
    check_roundtrip(m);

    // Pad up looks up, and stays up against gravity correction.
    switch2_motion_reset();
    switch2_motion_update(flat_accel, no_gyro, true, 0, 20000, 0.004f);
    for (int i = 0; i < 2000; i++) {
        switch2_motion_update(flat_accel, no_gyro, true, 0, 0, 0.004f);
        switch2_motion_advance(0.004f);
    }
    switch2_motion_get(&m);
    forward(m.q, f);
    float pitch = asinf(f[1]);
    assert(near(pitch, 20000 * PAD_RAD, 1e-3f));
    assert(near(m.accel[1], sinf(pitch), 1e-3f));  // Gravity agrees with the aim
    check_roundtrip(m);

    // Pad pitch stops at about 80 degrees.
    switch2_motion_update(flat_accel, no_gyro, true, 0, 200000, 0.004f);
    settle();
    switch2_motion_get(&m);
    forward(m.q, f);
    assert(near(asinf(f[1]), 1.4f, 1e-3f));

    // Smoothing: a small, slow pad move is eased in over a few reports...
    switch2_motion_reset();
    switch2_motion_update(flat_accel, no_gyro, true, -100, 0, 0.004f);
    switch2_motion_advance(0.004f);
    switch2_motion_get(&m);
    forward(m.q, f);
    float slow_target = 100 * PAD_RAD;
    assert(f[0] < -0.15f * slow_target && f[0] > -0.6f * slow_target);
    for (int i = 0; i < 12; i++) switch2_motion_advance(0.004f);
    switch2_motion_get(&m);
    forward(m.q, f);
    assert(f[0] < -0.95f * slow_target);

    // ...and a sudden flick is smoothed too, landing within a few reports.
    switch2_motion_reset();
    float flick = 40000 * PAD_RAD;
    switch2_motion_update(flat_accel, no_gyro, true, -40000, 0, 0.004f);
    switch2_motion_advance(0.004f);
    switch2_motion_get(&m);
    forward(m.q, f);
    assert(asinf(-f[0]) > 0.3f * flick && asinf(-f[0]) < 0.9f * flick);
    for (int i = 0; i < 3; i++) switch2_motion_advance(0.004f);
    switch2_motion_get(&m);
    forward(m.q, f);
    assert(asinf(-f[0]) > 0.9f * flick);

    // Uneven arrival (two pad samples, then none) comes out as even steps.
    switch2_motion_reset();
    float last = 0;
    float steps[40];
    for (int i = 0; i < 40; i++) {
        switch2_motion_update(flat_accel, no_gyro, true, i % 2 ? 0 : -200, 0, 0.004f);
        switch2_motion_advance(0.004f);
        switch2_motion_get(&m);
        forward(m.q, f);
        steps[i] = asinf(-f[0]) - last;
        last = asinf(-f[0]);
    }
    for (int i = 20; i < 40; i++) assert(near(steps[i], 100 * PAD_RAD, 0.5f * 100 * PAD_RAD));

    // Real gyro: 90 deg/s about the up axis for 1 s turns left by 90 degrees.
    switch2_motion_reset();
    const int16_t yaw_left[3] = { 0, 0, int16_t(90 * 32768 / 2000) };
    for (int i = 0; i < 250; i++) switch2_motion_update(flat_accel, yaw_left, true, 0, 0, 0.004f);
    switch2_motion_get(&m);
    forward(m.q, f);
    assert(near(f[0], -1, 0.01f) && near(m.gyro_dps[2], 90, 0.1f));

    // Real tilt with no gravity support drifts back to what gravity says.
    switch2_motion_reset();
    const int16_t roll[3] = { 0, int16_t(180 * 32768 / 2000), 0 };
    for (int i = 0; i < 50; i++) switch2_motion_update(flat_accel, roll, true, 0, 0, 0.004f);
    switch2_motion_get(&m);
    assert(m.accel[0] > 0.3f || m.accel[0] < -0.3f);  // Tilted about 36 degrees
    for (int i = 0; i < 2500; i++) switch2_motion_update(flat_accel, no_gyro, true, 0, 0, 0.004f);
    switch2_motion_get(&m);
    assert(near(m.accel[2], 1, 1e-3f));

    // Odd orientations still pack losslessly enough.
    for (int k = 0; k < 200; k++) {
        const int16_t spin[3] = { int16_t(k * 37 - 3000), int16_t(4000 - k * 29), int16_t(k * 11) };
        switch2_motion_update(flat_accel, spin, true, k * 13, -k * 7, 0.004f);
        switch2_motion_advance(0.004f);
        switch2_motion_get(&m);
        check_roundtrip(m);
    }
    printf("ok\n");
}
