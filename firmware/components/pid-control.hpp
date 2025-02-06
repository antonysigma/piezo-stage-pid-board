#pragma once

#include "callbacks.hpp"
#include "config.h"
#include "data-models/pid-events.h"

namespace components {
namespace pid_control {

namespace internal {
// Option to invert the system output.
constexpr bool INVERT_OUTPUT = true;
constexpr auto sign = INVERT_OUTPUT ? -1 : 1;

constexpr float Kp = 2e1f * sign;  // Proportional gain
constexpr float Ti = 1e-2f;        // Integral time / second
constexpr float Td = 5e-4f;        // Derivative time / second

// Slew rate limiter: limit changes to 50um / 5ms = 20 count / ms
constexpr float slewRatelimit = 20e3f;

constexpr float Ki_times_DeltaT = Kp / Ti * 1e-6f * sampleTime;
constexpr float Kd_over_DeltaT = Kp * Td * 1e6f / sampleTime;
constexpr float eMax = slewRatelimit * sampleTime * 1e-6f;  // Slew rate limit
constexpr uint16_t systemInputdefault = 3277;               // 4096U * 4 / 5;
}  // namespace internal

struct impl {
    constexpr static auto config = cib::config(cib::extend<SetDesiredSystemOutput>([](data_models::position_t desired_position) {

    });
};
}  // namespace pid_control
}  // namespace components