#pragma once

#include <Arduino.h>

#include "callbacks.hpp"
#include "components/linear-encoder.hpp"
#include "config.h"
#include "data-models/pid-events.h"
#include "pid-controller.h"

namespace components {
namespace pid_control {

namespace internal {

PIDController pid_controller{};
}  // namespace internal

struct impl {
    constexpr static auto config = cib::config(  //
        cib::extend<SetDesiredSystemOutput>([](data_models::position_t desired_position) {
            using UM16 = units::Micrometer<int16_t>;
            internal::pid_controller.setDesiredSystemOutput(UM16{desired_position.data.value});
        }),  //
        cib::extend<MainLoop>([]() {
            const auto current_time = micros();

            //! @todo This violates dependency inversion.
            internal::pid_controller.update(current_time,
                                            components::linear_encoder<encoder_A, encoder_B>::read);
        })  //
    );
};
}  // namespace pid_control
}  // namespace components