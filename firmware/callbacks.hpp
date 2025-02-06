#pragma once
#include <cib/cib.hpp>

#include "data-models/pid-events.h"
#include "data-models/position.h"

class MainLoop : public cib::callback_meta<> {};
class Alarm : public cib::callback_meta<data_models::system_input_t> {};
class PositionLock : public cib::callback_meta<data_models::system_error_t> {};
class PIDControl : public cib::callback_meta<data_models::position_t> {};