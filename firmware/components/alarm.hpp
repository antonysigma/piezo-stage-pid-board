#pragma once

#include <boost/sml.hpp>

// Include Arduino.h after sml.hpp
#include <Arduino.h>

#include "callbacks.hpp"
#include "components/core.hpp"
#include "config.h"
#include "data-models/dac-command.h"

namespace components {
namespace Alarm {

using data_models::dac_command_t;

namespace internal {
// Guards
constexpr auto systemInputExceedLimit = [](const dac_command_t t) -> bool {
    using S = decltype(dac_command_t::value);
    return t.value <= S(0) || t.value >= S(systemInputmax.value);
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
            "monitoring"_s + event<dac_command_t>[ systemInputExceedLimit ] / showAlarm = "monitoring"_s,
            "monitoring"_s + event<dac_command_t>[ not systemInputExceedLimit ] / silentAlarm = "monitoring"_s
            // clang-format on
        );
    }
};

using dispatch_t = boost::sml::dispatch<boost::sml::back::policies::branch_stm>;
boost::sml::sm<AlarmState, dispatch_t> alarm_state_machine;

}  // namespace internal

template <uint8_t alarm_led_pin>
struct impl {
    static constexpr auto init_alarm =
        flow::action("InitAlarm"_sc, []() { pinMode(alarm_led_pin, OUTPUT); });

    static void processEvent(dac_command_t event) {
        internal::alarm_state_machine.process_event(event);
    }

    constexpr static auto config = cib::config(cib::extend<RuntimeInit>(  //
        components::core::disable_usart >> init_alarm));
};

}  // namespace Alarm
}  // namespace components