#include "callbacks.hpp"
#include "components/alarm.hpp"
#include "components/command_parser.hpp"
#include "components/core.hpp"
#include "components/position-lock.hpp"

namespace {
struct interfaces {
    static constexpr auto config = cib::config(  //
        cib::exports<RuntimeInit>,               //
        cib::exports<MainLoop>,                  //
        cib::exports<PIDControl>,                //
        cib::exports<PositionLock>,
        cib::exports<Alarm>  //
    );
};

using namespace components;
struct project {
    static constexpr auto config = cib::components<  //
        interfaces,                                  //
        core::init,                                  //
        alarm::init,
        alarm::impl,           //
        command_parser::init,  //
        command_parser::impl,  //
        position_lock::init,   //
        position_lock::impl>;
};

}  // namespace
int
main() {
    cib::nexus<project> nexus{};

    nexus.service<RuntimeInit>();

    for (;;) {
        nexus.service<Alarm>(data_models::system_input_t{1.0f});
        nexus.service<MainLoop>();
    }

    return 0;
}