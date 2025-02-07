#pragma once
#include "units.hpp"

using namespace units::literals;

constexpr auto MCU_ADDR = 0x09;
constexpr auto DAC_ADDR = 0x63;

constexpr auto encoder_A = 2;
constexpr auto encoder_B = 3;
constexpr auto encoder_IDX = 4;
constexpr auto lockLED = 6;
constexpr auto alarmLED = 7;

constexpr uint16_t dac_offset =
    static_cast<uint16_t>(4096L * 4 / 5);  // 4096 steps / 5 Volt * 4.0 Volt;

constexpr auto systemInputmax = 4095_count;
constexpr auto sampleTime = 500_us;