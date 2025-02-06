#pragma once
#include <Encoder.h>

#include "callbacks.hpp"
#include "config.h"
#include "data-models/encoder-readout.h"

namespace components {
namespace linear_encoder {

namespace internal {
Encoder encoder(encoder_A, encoder_B);
}

struct impl {
    constexpr static auto config = cib::config(cib::extend<ResetPositionSensor>([]() {
        constexpr auto null_position = 0;
        internal::encoder.write(null_position);
    }));
};
}  // namespace linear_encoder
}  // namespace components