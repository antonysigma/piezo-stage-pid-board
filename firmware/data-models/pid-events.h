#pragma once
#include "units.hpp"

namespace data_models {

/** Difference between desired system output and the actual output. */
struct system_error_t {
    units::Count value{};
};

/** System input value. */
struct system_input_t {
    units::Count value{};
};

}  // namespace data_models