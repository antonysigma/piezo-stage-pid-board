#pragma once
#include <cib/cib.hpp>

#include "data-models/dac-command.h"
#include "data-models/pid-events.h"
#include "data-models/position.h"

// Main routine
class RuntimeInit : public flow::service<> {};
class MainLoop : public cib::callback_meta<> {};

// Indicators
class Alarm : public cib::callback_meta<data_models::system_input_t> {};
class PositionLock : public cib::callback_meta<data_models::system_error_t> {};

// Sensors
class ResetPositionSensor : public cib::callback_meta<> {};

// Actuators
class MoveTo : public cib::callback_meta<data_models::dac_command_t> {};

// Controllers
class PIDControl : public cib::callback_meta<data_models::position_t> {};