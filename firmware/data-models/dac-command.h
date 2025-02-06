#pragma once
#include <cstdint>

namespace data_models {
struct dac_command_t {
    uint16_t value{};

    constexpr dac_command_t(float voltage) : value{static_cast<uint16_t>(voltage)} {}
};
}  // namespace data_models