#include "pid-controller.h"

#include "config.h"
#include "utils/clamp.hpp"
PIDController::PIDController() {
    // previousMicros = micros();
}

void
PIDController::setDesiredSystemOutput(units::Micrometer<int16_t> value) {
    using M32 = units::Micrometer<int32_t>;
    x_desired = scale_factor * M32(value);
}

data_models::dac_command_t
PIDController::getSystemInput() const {
    return {static_cast<uint16_t>(u)};
}

units::Step
PIDController::getSystemOutput() const {
    return x_actual[0];
}
bool
PIDController::update(uint32_t currentMicros, readout_func encoder_readout_func) {
    using utils::clamp;
    if ((currentMicros - previousMicros) < sampleTime.value) {
        return false;
    }

    previousMicros = currentMicros;

    const auto new_x_actual = encoder_readout_func().value;
    // Compute system input error with slew rate limiter
    const float new_e = clamp(x_desired - new_x_actual, -eMax, eMax).value;

    const float compensated =
        u  //
           // Apply P gain
        + Kp * (new_e - e)  //

        // Apply I gain
        + Ki_times_DeltaT * new_e

        // Apply D gain based on system output only
        + Kd_over_DeltaT * (new_x_actual.value - x_actual[0].value * 2.0f + x_actual[1].value);

    // Prevent integral windup
    // Show alarm when system input limit is reached
    const auto clamped = clamp(compensated, 0.0f, float(systemInputmax.value));

    // Apply another slew rate limiter
    const auto new_u = 0.97f * u + 0.03f * clamped;

    // Apply system input, only when there is a significant change
    // Used to reduce i2c traffic
    using units::Abs;
    const bool has_significant_change = Abs(new_u - u) > 1.0f;

    // Update state variables
    x_actual[1] = x_actual[0];
    x_actual[0] = new_x_actual;
    e = new_e;

    return has_significant_change;
}
