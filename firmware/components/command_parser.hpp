#pragma once

#include <Wire.h>

#include "callbacks.hpp"
#include "components/core.hpp"
#include "config.h"
#include "data-models/position.h"
#include "utils/clamp.hpp"

namespace components {
namespace command_parser {

using data_models::position_t;

namespace internal {
volatile position_t position_command;
volatile bool has_new_position{false};
}  // namespace internal

static constexpr auto setup_i2c = flow::action("I2CInit"_sc, []() {
    // Register the MCU in multi-master mode, having I2C address 0x09.
    Wire.begin(MCU_ADDR);
    Wire.onReceive(cib::service<OnIncomingMessage>);
});

template <class Controller>
struct impl {
    constexpr static auto config =
        cib::config(cib::extend<RuntimeInit>(                           //
                        components::core::disable_usart >> setup_i2c),  //
                    cib::extend<OnIncomingMessage>([](int) {
                        auto& buffer = internal::position_command.data.buffer;
                        buffer[0] = Wire.read();
                        buffer[1] = Wire.read();
                        internal::has_new_position = true;
                    }),  //
                    cib::extend<MainLoop>([]() {
                        using internal::has_new_position;
                        using internal::position_command;
                        using utils::clamp;
                        if (has_new_position) {
                            //! @todo Explicit casting defeats the purpose of SI units.
                            const int16_t retrieved_value = position_command.data.value;

                            using Micron = units::Micrometer<int16_t>;
                            Controller::setDesiredSystemOutput(Micron{retrieved_value});
                            has_new_position = false;
                        }
                    }));
};

}  // namespace command_parser
}  // namespace components
