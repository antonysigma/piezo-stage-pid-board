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
struct interfaces {
    static constexpr auto config = cib::config(  //
        cib::exports<RuntimeInit>,               //
        cib::exports<MainLoop>,                  //
        cib::exports<SetDesiredSystemOutput>,    //
        cib::exports<TestPositionLock>,          //
        cib::exports<MoveTo>,
        cib::exports<TestPIDFault>,            //
        cib::exports<SetDesiredSystemOutput>,  //
        cib::exports<ResetPositionSensor>);
};

using namespace components;
struct project {
    static constexpr auto config = cib::components<  //
        interfaces,                                  //
        core::impl,                                  //
        alarm::impl,                                 //
        command_parser::impl,                        //
        position_lock::impl,                         //
        linear_encoder::impl,                        //
        pid_control::impl,                           //
        actuator::impl                               //
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