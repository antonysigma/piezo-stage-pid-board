#pragma once
#include "boost/sml.hpp"

// Include Arduino.h after sml.hpp
#include <Arduino.h>

#include "config.h"

namespace state_machine {

namespace lock {

constexpr float thresh = 1.0f;

// Events
struct system_error_t {
    float value = 0.0f;
};

// States
struct tracking {
    uint8_t n_debounce = 0;
};

// Guards
constexpr auto errorWithinThreshold = [](const system_error_t t) -> bool {
    return abs(t.value) <= thresh;
};

constexpr auto systemInputExceedLimit = [](const system_error_t t) -> bool {
    return t.value <= 0 || t.value >= systemInputmax;
};

// Debounce switch: change state only after 50ms
constexpr uint8_t max_debounce = 5000UL / sampleTime;

constexpr auto debounceReady = [](const system_error_t, const tracking t) -> bool {
    return t.n_debounce >= max_debounce;
};

// Actions
constexpr auto setupLock = []() { pinMode(lockLED, OUTPUT); };
constexpr auto showLock = []() { digitalWrite(lockLED, HIGH); };
constexpr auto showUnlock = []() { digitalWrite(lockLED, LOW); };

constexpr auto debounce = [](const system_error_t, tracking& t) { t.n_debounce++; };
constexpr auto reset = [](const system_error_t, tracking& t) { t.n_debounce = 0; };

struct LockState {
    auto operator()() {
        using namespace boost::sml;

        return make_transition_table(
            // clang-format off
            *"init"_s / setupLock = state<tracking>,
            state<tracking> + event<system_error_t>[errorWithinThreshold and not debounceReady] / debounce = state<tracking>,
            state<tracking> + event<system_error_t>[errorWithinThreshold and debounceReady] / showLock = "locked"_s,
            "locked"_s + event<system_error_t>[not errorWithinThreshold] / (showUnlock, reset) = state<tracking>
            // clang-format on
        );
    }
};

}  // namespace lock
}  // namespace state_machine