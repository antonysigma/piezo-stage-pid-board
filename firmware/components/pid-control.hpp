#pragma once

#include <Arduino.h>

#include "callbacks.hpp"
#include "components/linear-encoder.hpp"
#include "config.h"
#include "data-models/pid-events.h"
#include "pid-controller.h"

namespace components {
namespace pid_control {

using Micron = units::Micrometer<int16_t>;

namespace internal {
PIDController controller{};
}

template <Micron z_min, Micron z_max, class PositionSensor, class ZStage>
struct impl {
    static constexpr void setDesiredSystemOutput(const units::Micrometer<int16_t> value) {
        using utils::clamp;
        const auto clamped_position = clamp(value, z_min, z_max);
        internal::controller.setDesiredSystemOutput(clamped_position);
    }

    constexpr static auto config = cib::config(  //
        cib::extend<MainLoop>([]() {
            const auto current_time = micros();

            const bool has_significant_change =
                internal::controller.update(current_time, PositionSensor::read);
            if (has_significant_change) {
                ZStage::moveTo(internal::controller.getSystemInput());
            }
        })  //
    );
};
}  // namespace pid_control
}  // namespace components