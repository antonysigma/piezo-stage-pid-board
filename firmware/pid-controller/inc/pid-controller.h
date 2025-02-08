#pragma once
#include <stdint.h>

#include "config.h"
#include "utils/units.hpp"

// todo: Why does PIDControl needs to know about encoder and DAC scale factors?
#include "data-models/dac-command.h"
#include "data-models/encoder-readout.h"

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

    static constexpr auto scale_factor = 2_step / 1_um;

    //! Slew rate limiter: limit changes to 50um / 5ms = 20 count / us
    static constexpr auto slewRatelimit = scale_factor * (50_um / 5_ms);

    //! @todo Why the hardware-in-the-loop experiment indicates 1000x difference?
    // static_assert(slewRatelimit.value == 20e3f);

    static constexpr float Ki_times_DeltaT = Kp / (Ti / sampleTime);
    static constexpr float Kd_over_DeltaT = Kp * (Td / sampleTime);
    static constexpr auto eMax = slewRatelimit * (1_ms / 1'000_us) * sampleTime;  // Slew rate limit
    static_assert(eMax > 1_step);

    static constexpr uint16_t systemInputdefault = dac_offset;

   private:
    units::Step x_desired{0};          // desired output
    units::Step x_actual[2] = {0, 0};  // actual output

    float e = 0;                   // Previous error value
    float u = systemInputdefault;  // Previous control input

    uint32_t previousMicros;  // will store last time LED was updated

   public:
    PIDController();

    void setDesiredSystemOutput(units::Micrometer<int16_t>);

    //! @todo Should implement dependency injection here.
    [[nodiscard]] data_models::dac_command_t getSystemInput() const;
    [[nodiscard]] units::Step getSystemOutput() const;

    using readout_func = data_models::encoder_readout_t (*)();
    bool update(uint32_t currentMicros, readout_func);
};
