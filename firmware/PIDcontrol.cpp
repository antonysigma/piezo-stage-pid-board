#include "alarm-state-logic.hpp"
#include "lock-state-logic.hpp"

#include "PIDcontrol.h"

// Include the rest of the headers
#include <Arduino.h>

#include "config.h"
#include "utils.hpp"

PID_control::PID_control(Adafruit_MCP4725* _dac, Encoder* _encoder) : dac(_dac), encoder(_encoder) {
    pinMode(encoder_IDX, INPUT_PULLUP);

    previousMicros = micros();
}

void
PID_control::begin() {
    delay(200);

    // TODO: limit the slew rate
    uint16_t position = systemInputdefault + 200;
    dac->setVoltage(position, false);

    delay(200);

    // Scan for index signal
    for (; position > systemInputdefault - 200; position--) {
        // TODO: Limit slew rate to 1 count / ms
        dac->setVoltage(position, false);
        delay(10);

        if (!digitalRead(encoder_IDX)) break;
    }

    encoder->write(0);
}

void
PID_control::setSystemoutput(int16_t value) {
    // Gain = 2 count / micrometer
    x_desired = value * 2;
}

uint16_t
PID_control::getSysteminput() const {
    return u;
}

int16_t
PID_control::getSystemoutput() const {
    return x_actual[0];
}
void
PID_control::update(unsigned long currentMicros) {
    using utils::clamp;
    if (currentMicros - previousMicros < sampleTime) return;

    using dispatch_t = boost::sml::dispatch<boost::sml::back::policies::branch_stm>;
    using AlarmState = state_machine::alarm::AlarmState;
    using LockState = state_machine::lock::LockState;
    static boost::sml::sm<AlarmState, dispatch_t> alarm_state_machine;
    static boost::sml::sm<LockState, dispatch_t> lock_state_transition;

    previousMicros = currentMicros;

    const int16_t new_x_actual = encoder->read();
    // Compute system input error with slew rate limiter
    const float new_e = clamp(float(x_desired) - new_x_actual, -eMax, eMax);

    // Apply PI gain
    float new_u = u;

    new_u += Kp * (new_e - e);
    new_u += Ki_times_DeltaT * new_e;

    // Apply D gain based on system output only
    new_u += Kd_over_DeltaT * (new_x_actual - x_actual[0] * 2 + x_actual[1]);

    // Prevent integral windup
    // Show alarm when system input limit is reached
    new_u = clamp(new_u, 0.0f, float(systemInputmax));

    // Apply another slew rate limiter
    u = 0.97f * u + 0.03f * new_u;

    // Apply system input, only when there is a significant change
    // Used to reduce i2c traffic
    if (u - new_u > 1 || u - new_u < -1) dac->setVoltage(static_cast<uint16_t>(u), false);

    // Update state variables
    x_actual[1] = x_actual[0];
    x_actual[0] = new_x_actual;
    e = new_e;

    // Visualize the lock state with LEDs
    using state_machine::alarm::system_input_t;
    using state_machine::lock::system_error_t;
    lock_state_transition.process_event(system_error_t{new_e});
    alarm_state_machine.process_event(system_input_t{new_u});
}
