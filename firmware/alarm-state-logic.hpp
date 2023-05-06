#pragma once
#include "boost/sml.hpp"

// Include Arduino.h after sml.hpp
#include <Arduino.h>

#include "config.h"

namespace state_machine {

namespace alarm {

// Events
struct system_input_t {
    float value = 0;
};

// Guards
constexpr auto systemInputExceedLimit = [](const system_input_t t) -> bool {
    return t.value <= 0 || t.value >= systemInputmax;
};

// Actions
constexpr auto setupAlarm = []() { pinMode(alarmLED, OUTPUT); };
constexpr auto showAlarm = []() { digitalWrite(alarmLED, HIGH); };
constexpr auto silentAlarm = []() { digitalWrite(alarmLED, LOW); };

struct AlarmState {
    auto operator()() const {
        using namespace boost::sml;
        return make_transition_table(
            // clang-format off
            *"init"_s / setupAlarm = "monitoring"_s,
            "monitoring"_s + event<system_input_t>[ systemInputExceedLimit ] / showAlarm = "monitoring"_s,
            "monitoring"_s + event<system_input_t>[ not systemInputExceedLimit ] / silentAlarm = "monitoring"_s
            // clang-format on
        );
    }
};

}  // namespace alarm
}  // namespace state_machine