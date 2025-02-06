#pragma once
#include <Encoder.h>

#include "callbacks.hpp"
#include "config.h"
#include "data-models/encoder-readout.h"
#include "units.hpp"

namespace components {
namespace linear_encoder {

namespace internal {
Encoder encoder(encoder_A, encoder_B);
}

units::Count
read() {
    return {internal::encoder.read()};
}

struct impl {
    constexpr static auto config = cib::config(cib::extend<ResetPositionSensor>([]() {
        constexpr auto null_position = 0;
        internal::encoder.write(null_position);
    }));
};
}  // namespace linear_encoder
}  // namespace components