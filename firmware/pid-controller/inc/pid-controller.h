#pragma once
#include <stdint.h>

#include "config.h"
#include "units.hpp"

using namespace units::literals;
using units::Abs;

class PIDController {
   public:
    // Option to invert the system output.
    static constexpr bool INVERT_OUTPUT = true;
    static constexpr auto sign = INVERT_OUTPUT ? -1 : 1;

    static constexpr float Kp = 2e1f * sign;  // Proportional gain
    static constexpr auto Ti = 10'000_us;     // Integral time
    static constexpr auto Td = 500_us;        // Derivative time

    // Slew rate limiter: limit changes to 50um / 5ms = 20 count / ms
    static constexpr auto scale_factor = 2_count / 1_um;
    static constexpr auto slewRatelimit = scale_factor * (50_um / 5_ms) * (1_ms / 1000_us);

    // Bug: Should be 20 count / ms instead.
    // static_assert(slewRatelimit.value == 20e3f);

    static constexpr float Ki_times_DeltaT = Kp / (Ti / sampleTime);
    static constexpr float Kd_over_DeltaT = Kp * (Td / sampleTime);
    static constexpr auto eMax = slewRatelimit * sampleTime;  // Slew rate limit
    static constexpr uint16_t systemInputdefault = dac_offset;

   private:
    units::Count x_desired{0};          // desired output
    units::Count x_actual[2] = {0, 0};  // actual output

    float e = 0;                   // Previous error value
    float u = systemInputdefault;  // Previous control input

    uint32_t previousMicros;  // will store last time LED was updated

   public:
    PIDController();

    void setDesiredSystemOutput(units::Micrometer);
    [[nodiscard]] uint16_t getSysteminput() const;
    [[nodiscard]] units::Count getSystemOutput() const;

    void update(uint32_t currentMicros);
};
