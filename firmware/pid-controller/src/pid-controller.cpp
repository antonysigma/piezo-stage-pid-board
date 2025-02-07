#include "pid-controller.h"

#include "config.h"
#include "utils.hpp"
PIDController::PIDController() {
    // previousMicros = micros();
}

void
PIDController::setDesiredSystemOutput(units::Micrometer value) {
    x_desired = scale_factor * value;
}

data_models::dac_command_t
PIDController::getSystemInput() const {
    return {static_cast<uint16_t>(u)};
}

units::Step
PIDController::getSystemOutput() const {
    return x_actual[0];
}
void
PIDController::update(uint32_t currentMicros, data_models::encoder_readout_t encoder_readout) {
    using utils::clamp;
    if ((currentMicros - previousMicros) < sampleTime.value) return;

    previousMicros = currentMicros;

    // const int16_t new_x_actual = encoder->read();
    const auto new_x_actual = encoder_readout.value;
    // Compute system input error with slew rate limiter
    const float new_e = clamp(x_desired - new_x_actual, -eMax, eMax).value;

    // Apply PI gain
    float new_u = u;

    new_u += Kp * (new_e - e);
    new_u += Ki_times_DeltaT * new_e;

    // Apply D gain based on system output only
    new_u += Kd_over_DeltaT * (new_x_actual.value - x_actual[0].value * 2.0f + x_actual[1].value);

    // Prevent integral windup
    // Show alarm when system input limit is reached
    new_u = clamp(new_u, 0.0f, float(systemInputmax.value));

    // Apply another slew rate limiter
    u = 0.97f * u + 0.03f * new_u;

    // Apply system input, only when there is a significant change
    // Used to reduce i2c traffic
    // if (u - new_u > 1 || u - new_u < -1) {
    // dac->setVoltage(static_cast<uint16_t>(u), false);
    //}

    // Update state variables
    x_actual[1] = x_actual[0];
    x_actual[0] = new_x_actual;
    e = new_e;
}
