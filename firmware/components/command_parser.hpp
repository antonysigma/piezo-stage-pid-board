#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "callbacks.hpp"
#include "components/core.hpp"
#include "config.h"
#include "data-models/position.h"
#include "utils.hpp"

namespace components {
namespace command_parser {

using data_models::position_t;
namespace internal {

volatile position_t position_command;
volatile bool has_new_position{false};

void
receiveEvent(int numBytes) {
    auto& buffer = position_command.data.buffer;
    buffer[0] = Wire.read();  // receive byte as a character
    buffer[1] = Wire.read();
    has_new_position = true;
}

}  // namespace internal

static constexpr auto setup_i2c = flow::action("I2CInit"_sc, []() {
    // Register the MCU in multi-master mode, having I2C address 0x09.
    Wire.begin(MCU_ADDR);
    Wire.onReceive(internal::receiveEvent);
});

struct init {
    constexpr static auto config = cib::config(cib::extend<RuntimeInit>(  //
        components::core::disable_usart >> setup_i2c));
};

struct impl {
    constexpr static auto config = cib::config(cib::extend<MainLoop>([]() {
        using internal::has_new_position;
        using internal::position_command;
        using utils::clamp;
        if (has_new_position) {
            // When the new position command is received, execute it.
            data_models::position_t clamped_position{};
            clamped_position.data.value = clamp(position_command.data.value, -50, 250);

            // How do I send a message to another component?
            cib::service<PIDControl>(clamped_position);

            has_new_position = false;
        }
    }));
};

}  // namespace command_parser
}  // namespace components
