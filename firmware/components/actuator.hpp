#pragma once

#include <Adafruit_MCP4725.h>
#include <Wire.h>

#include "components/command_parser.hpp"

namespace components {

template <uint8_t dac_addr>
struct actuator {
    static Adafruit_MCP4725 dac;

    static constexpr auto setup_dac_connection = flow::action("DACInit"_sc, []() {
        // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
        // For MCP4725A0 the address is 0x60 or 0x61
        // For MCP4725A2 the address is 0x64 or 0x65
        dac.begin(dac_addr, &Wire);
    });

    static constexpr auto setup_idx_input =
        flow::action("IDXInit"_sc, []() { pinMode(encoder_IDX, INPUT_PULLUP); });

    static constexpr auto search_idx = flow::action("SearchIDX"_sc, []() {
        delay(200);

        // Move to neutral position
        constexpr uint16_t search_range = 200;
        uint16_t position = dac_offset + search_range;
        dac.setVoltage(position, false);

        // Scan for index signal
        for (; position > dac_offset - search_range; position--) {
            // TODO(Antony): Limit the slew rate to 1 count / ms
            dac.setVoltage(position, false);
            delay(10);

            if (!digitalRead(encoder_IDX)) {
                break;
            }
        }

        cib::service<ResetPositionSensor>();
    });

    static constexpr void moveTo(data_models::dac_command_t position) {
        dac.setVoltage(position.value, false);
    }

    constexpr static auto config = cib::config(  //
        cib::extend<RuntimeInit>(components::command_parser::setup_i2c >> setup_idx_input >>
                                 setup_dac_connection >> search_idx)  //
    );
};

}  // namespace components
