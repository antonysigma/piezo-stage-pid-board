#pragma once

#include <cstdint>

namespace data_models {
#pragma pack(push, 1)
struct position_t {
    union {
        uint16_t value;
        uint8_t buffer[2];
    } data;
};
#pragma pack(pop)
}  // namespace data_models