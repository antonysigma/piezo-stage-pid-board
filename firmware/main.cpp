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

//! @todo Why an out-of-class instantiation of static member?
using my_actuator = components::actuator<DAC_ADDR>;
using pid_controller_impl = components::pid_control::impl<z_min, z_max>;

using D = decltype(my_actuator::dac);
template <>
D my_actuator::dac{};

namespace {
struct registered_interfaces {
    static constexpr auto config = cib::config(  //
        cib::exports<RuntimeInit>,               //
        cib::exports<MainLoop>,                  //
        cib::exports<TestPositionLock>,          //
        cib::exports<MoveTo>,
        cib::exports<TestPIDFault>,         //
        cib::exports<ResetPositionSensor>,  //
        cib::exports<OnIncomingMessage>     //
    );
};

using namespace components;
struct project {
    static constexpr auto config = cib::components<  //
        registered_interfaces,                       //
        core::impl,                                  //
        // Indicators
        alarm::impl<alarmLED>,         //
        position_lock::impl<lockLED>,  //

        // Sensors
        linear_encoder<encoder_A, encoder_B>,  //

        // Actuators
        my_actuator,  //

        // Command dispatcher
        command_parser::impl<pid_controller_impl>,  //

        // Controllers
        pid_controller_impl  //
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