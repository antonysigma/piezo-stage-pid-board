// Boost-SML must be included before Arduino.h
#include <boost/sml.hpp>

#include "callbacks.hpp"
#include "components/actuator.hpp"
#include "components/alarm.hpp"
#include "components/command_parser.hpp"
#include "components/core.hpp"
#include "components/linear-encoder.hpp"
#include "components/pid-control.hpp"
#include "components/position-lock.hpp"

namespace {
struct registered_interfaces {
    static constexpr auto config = cib::config(  //
        cib::exports<RuntimeInit>,               //
        cib::exports<MainLoop>,                  //
        cib::exports<OnIncomingMessage>          //
    );
};

using namespace components;

// Sensors
using linear_encoder = LinearEncoder<encoder_A, encoder_B>;

// Actuators
using actuator = Actuator<DAC_ADDR, linear_encoder>;

// Indicators
using alarm = Alarm::impl<alarmLED>;
using position_lock = PositionLock::impl<lockLED>;

// Controllers
using pid_controller_impl =
    pid_control::impl<z_min, z_max, linear_encoder, actuator, alarm, position_lock>;

struct project {
    static constexpr auto config = cib::components<  //
        registered_interfaces,                       //
        core::impl,                                  //
        alarm,                                       //
        actuator,                                    //

        // Command dispatcher
        command_parser::impl<pid_controller_impl>,  //
        pid_controller_impl                         //
        >;
};

}  // namespace

int
main() {
    cib::nexus<project> nexus{};

    nexus.service<RuntimeInit>();

    for (;;) {
        nexus.service<MainLoop>();
    }

    return 0;
}