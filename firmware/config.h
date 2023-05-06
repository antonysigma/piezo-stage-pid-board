#pragma once

constexpr auto MCU_ADDR = 0x09;
constexpr auto DAC_ADDR = 0x63;

constexpr auto encoder_A = 2;
constexpr auto encoder_B = 3;
constexpr auto encoder_IDX = 4;
constexpr auto lockLED = 6;
constexpr auto alarmLED = 7;

constexpr uint16_t dac_offset = 3277;  // 4096L / 5Volt * 4Volt

constexpr uint16_t systemInputmax = 4095;
constexpr auto sampleTime = 500UL;