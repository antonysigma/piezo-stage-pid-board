#pragma once

// #include <boost/sml.hpp>

// Include Arduino.h after sml.hpp
#include <Arduino.h>

#include "components/core.hpp"
#include "config.h"
#include "data-models/pid-events.h"

namespace components {
namespace alarm {

static constexpr auto setup_pin = flow::action("AlarmInit"_sc, []() { pinMode(alarmLED, OUTPUT); });

struct init {
    constexpr static auto config = cib::config(cib::extend<RuntimeInit>(  //
        components::core::disable_usart >> setup_pin));
};

// Events
using data_models::system_input_t;

// Guards
constexpr auto systemInputExceedLimit = [](const system_input_t t) -> bool {
    return t.value <= 0 || t.value >= systemInputmax;
};

// Actions
constexpr auto showAlarm = []() { digitalWrite(alarmLED, HIGH); };
constexpr auto silentAlarm = []() { digitalWrite(alarmLED, LOW); };

}  // namespace alarm
}  // namespace components