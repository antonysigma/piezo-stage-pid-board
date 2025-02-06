#pragma once
#include <cib/cib.hpp>

#include "data-models/pid-events.h"

class MainLoop : public cib::callback_meta<> {};
class Alarm : public cib::callback_meta<data_models::system_input_t> {};
