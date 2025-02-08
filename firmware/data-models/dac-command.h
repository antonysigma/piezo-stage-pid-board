#pragma once
#include "utils/units.hpp"

namespace data_models {
struct dac_command_t {
    units::Step<uint16_t> value{};

    // constexpr dac_command_t(units::Volt volt) : value{static_cast<uint16_t>(volt.value)} {}
};
}  // namespace data_models