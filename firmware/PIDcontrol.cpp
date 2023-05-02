#include "PIDcontrol.h"

#include <Arduino.h>

#include "config.h"

namespace {
constexpr uint16_t systemInputmax = 4095;
}  // namespace

PID_control::PID_control(Adafruit_MCP4725* _dac, Encoder* _encoder) : dac(_dac), encoder(_encoder) {
    pinMode(lockLED, OUTPUT);
    pinMode(alarmLED, OUTPUT);
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
    if (currentMicros - previousMicros < sampleTime) return;

    previousMicros = currentMicros;

    int16_t new_x_actual = encoder->read();
    float new_e = float(x_desired) - new_x_actual;
    bool lock_flag = (new_e <= 1 && new_e >= -1);

    // Slew rate limiter
    if (new_e > eMax)
        new_e = eMax;
    else if (new_e < -eMax)
        new_e = -eMax;

    // Apply PI gain
    float new_u = u;

    new_u += Kp * (new_e - e);
    new_u += Ki_times_DeltaT * new_e;

    // Apply D gain based on system output only
    new_u += Kd_over_DeltaT * (new_x_actual - x_actual[0] * 2 + x_actual[1]);

    // Prevent integral windup
    // Show alarm when system input limit is reached
    if (new_u < 0) {
        new_u = 0;
        digitalWrite(alarmLED, HIGH);
    } else if (new_u > systemInputmax) {
        new_u = systemInputmax;
        digitalWrite(alarmLED, HIGH);
    } else
        digitalWrite(alarmLED, LOW);

    // Apply another slew rate limiter
    u = 0.97 * u + 0.03 * new_u;

    // Apply system input, only when there is a significant change
    // Used to reduce i2c traffic
    if (u - new_u > 1 || u - new_u < -1) dac->setVoltage((uint16_t)u, false);

    // Update state variables
    x_actual[1] = x_actual[0];
    x_actual[0] = new_x_actual;
    e = new_e;

    // Show lock signal
    // Debounce switch: change state only after 50ms
    if (lock_flag && lockTimer > 50000UL)
        digitalWrite(lockLED, HIGH);
    else if (!lock_flag) {
        lockTimer = 0;
        digitalWrite(lockLED, LOW);
    } else
        lockTimer += sampleTime;
}
