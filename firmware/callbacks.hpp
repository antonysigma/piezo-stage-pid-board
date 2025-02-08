#pragma once
#include <cib/cib.hpp>

#include "data-models/dac-command.h"
#include "data-models/pid-events.h"
#include "data-models/position.h"

// Main routine
class RuntimeInit : public flow::service<> {};
class MainLoop : public cib::callback_meta<> {};

// Incoming i2c messages
class OnIncomingMessage : public cib::callback_meta<int> {};