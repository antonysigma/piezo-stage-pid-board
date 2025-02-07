#pragma once

#include <boost/sml.hpp>

// Include Arduino.h after sml.hpp
#include <Arduino.h>

#include "callbacks.hpp"
#include "components/core.hpp"
#include "config.h"
#include "data-models/pid-events.h"

namespace components {
namespace position_lock {

using data_models::system_error_t;

namespace internal {
// States
struct tracking {
    uint8_t n_debounce{};
};

// Guards
constexpr auto errorWithinThreshold = [](const system_error_t t) -> bool {
    return (-1_count <= t.value) && (t.value <= 1_count);
};

constexpr auto systemInputExceedLimit = [](const system_error_t t) -> bool {
    return t.value <= 0_count || t.value >= systemInputmax;
};

// Debounce switch: change state only after 50ms
constexpr uint8_t max_debounce = 5000_us / sampleTime;

constexpr auto debounceReady = [](const system_error_t, const tracking t) -> bool {
    return t.n_debounce >= max_debounce;
};

// Actions
constexpr auto showLock = []() { digitalWrite(lockLED, HIGH); };
constexpr auto showUnlock = []() { digitalWrite(lockLED, LOW); };

constexpr auto debounce = [](const system_error_t, tracking& t) { t.n_debounce++; };
constexpr auto reset = [](const system_error_t, tracking& t) { t.n_debounce = 0; };

struct LockState {
    auto operator()() {
        using namespace boost::sml;

        return make_transition_table(
            // clang-format off
            *"init"_s = state<tracking>,
            state<tracking> + event<system_error_t>[errorWithinThreshold and not debounceReady] / debounce = state<tracking>,
            state<tracking> + event<system_error_t>[errorWithinThreshold and debounceReady] / showLock = "locked"_s,
            "locked"_s + event<system_error_t>[not errorWithinThreshold] / (showUnlock, reset) = state<tracking>
            // clang-format on
        );
    }
};

using dispatch_t = boost::sml::dispatch<boost::sml::back::policies::branch_stm>;
boost::sml::sm<LockState, dispatch_t> position_lock_state_machine{};

}  // namespace internal

static constexpr auto setup_pin =
    flow::action("PositionLockInit"_sc, []() { pinMode(lockLED, OUTPUT); });

struct impl {
    constexpr static auto config =
        cib::config(cib::extend<RuntimeInit>(                           //
                        components::core::disable_usart >> setup_pin),  //
                    cib::extend<TestPositionLock>([](system_error_t event) {
                        internal::position_lock_state_machine.process_event(event);
                    })  //
        );
};

}  // namespace position_lock
}  // namespace components