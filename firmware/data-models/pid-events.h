#pragma once
#include "utils/units.hpp"

namespace data_models {

/** Difference between desired system output and the actual output. */
struct system_error_t {
    units::Step<int32_t> value{};
};

}  // namespace data_models