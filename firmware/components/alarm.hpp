#pragma once

#include <boost/sml.hpp>

// Include Arduino.h after sml.hpp
#include <Arduino.h>

#include "callbacks.hpp"
#include "components/core.hpp"
#include "config.h"
#include "data-models/pid-events.h"

namespace {
// Events
using data_models::system_input_t;

// Guards
constexpr auto systemInputExceedLimit = [](const system_input_t t) -> bool {
    return t.value <= 0 || t.value >= systemInputmax;
};

// Actions
constexpr auto showAlarm = []() { digitalWrite(alarmLED, HIGH); };
constexpr auto silentAlarm = []() { digitalWrite(alarmLED, LOW); };

struct AlarmState {
    auto operator()() const {
        using namespace boost::sml;
        return make_transition_table(
            // clang-format off
            *"init"_s = "monitoring"_s,
            "monitoring"_s + event<system_input_t>[ systemInputExceedLimit ] / showAlarm = "monitoring"_s,
            "monitoring"_s + event<system_input_t>[ not systemInputExceedLimit ] / silentAlarm = "monitoring"_s
            // clang-format on
        );
    }
};

using dispatch_t = boost::sml::dispatch<boost::sml::back::policies::branch_stm>;
boost::sml::sm<AlarmState, dispatch_t> alarm_state_machine;

}  // namespace

namespace components {
namespace alarm {

static constexpr auto setup_pin = flow::action("AlarmInit"_sc, []() { pinMode(alarmLED, OUTPUT); });

struct init {
    constexpr static auto config = cib::config(cib::extend<RuntimeInit>(  //
        components::core::disable_usart >> setup_pin));
};

struct impl {
    constexpr static auto config = cib::config(
        cib::extend<Alarm>([](system_input_t event) { alarm_state_machine.process_event(event); }));
};

}  // namespace alarm
}  // namespace components