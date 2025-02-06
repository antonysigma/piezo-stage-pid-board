#pragma once

namespace data_models {

/** Difference between desired system output and the actual output. */
struct system_error_t {
    float value{0.0f};
};

/** System input value. */
struct system_input_t {
    float value{0.0f};
};

}  // namespace data_models