#pragma once
#include <cib/cib.hpp>

// Main routine
class RuntimeInit : public flow::service<> {};
class MainLoop : public cib::callback_meta<> {};

// Incoming i2c messages
class OnIncomingMessage : public cib::callback_meta<int> {};