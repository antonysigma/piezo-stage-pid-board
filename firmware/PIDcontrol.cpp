// This header must come first
#include "boost/sml.hpp"

// Include the rest of the headers
#include <Arduino.h>

#include "PIDcontrol.h"
#include "config.h"

namespace {

constexpr float thresh = 1.0f;
constexpr uint16_t systemInputmax = 4095;

template <typename T>
constexpr T
clamp(const T x, const T vmin, const T vmax) {
    if (x < vmin) {
        return vmin;
    }

    if (x > vmax) {
        return vmax;
    }

    return x;
}

// Events
struct trigger {
    float error = 0.0f;
};

// Guards
constexpr auto errorWithinThreshold = [](const trigger t) -> bool {
    return abs(t.error) <= thresh;
};

constexpr auto setupLock = []() { pinMode(lockLED, OUTPUT); };
constexpr auto showLock = []() { digitalWrite(lockLED, HIGH); };
constexpr auto showUnlock = []() { digitalWrite(lockLED, LOW); };

struct LockState {
    // Debounce switch: change state only after 50ms
    static constexpr uint8_t max_debounce = 5000UL / PID_control::sampleTime;
    uint8_t n_debounce = 0;

    auto operator()() {
        using namespace boost::sml;

        // Actions
        const auto debounceReady = [&]() -> bool { return n_debounce >= max_debounce; };
        const auto debounce = [&]() { n_debounce++; };
        const auto reset = [&]() { n_debounce = 0; };

        return make_transition_table(
            // clang-format off
            *"init"_s / setupLock = "tracking"_s,
            "tracking"_s + event<trigger>[errorWithinThreshold and not debounceReady] / debounce = "tracking"_s,
            "tracking"_s + event<trigger>[errorWithinThreshold and debounceReady] / showLock = "locked"_s,
            "locked"_s + event<trigger>[not errorWithinThreshold] / (showUnlock, reset) = "tracking"_s
            // clang-format on
        );
    }
};

boost::sml::sm<LockState> lock_state_transition;

}  // namespace

PID_control::PID_control(Adafruit_MCP4725* _dac, Encoder* _encoder) : dac(_dac), encoder(_encoder) {
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

    // Visualize the lock state with LEDs
    lock_state_transition.process_event(trigger{new_e});
}
