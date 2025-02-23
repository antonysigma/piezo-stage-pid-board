#pragma once
#include <Encoder.h>

#include "callbacks.hpp"
#include "data-models/encoder-readout.h"
#include "utils/units.hpp"

namespace components {

template <uint8_t A_pin, uint8_t B_pin>
class LinearEncoder {
    static inline Encoder encoder{A_pin, B_pin};

   public:
    static data_models::encoder_readout_t read() { return {encoder.read()}; }
    static void reset() {
        constexpr auto null_position = 0;
        encoder.write(null_position);
    }
};
}  // namespace components