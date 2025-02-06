#include "callbacks.hpp"
#include "components/alarm.hpp"
#include "components/command_parser.hpp"
#include "components/core.hpp"

namespace {
struct harness {
    static constexpr auto config = cib::config(  //
        cib::exports<RuntimeInit>,               //
        cib::exports<MainLoop>,                  //
        cib::exports<PIDControl>,
        cib::exports<Alarm>  //
    );
};

using namespace components;
struct project {
    static constexpr auto config = cib::components<  //
        harness, core::init,                         //
        alarm::init,
        alarm::impl,           //
        command_parser::init,  //
        command_parser::impl   //
        >;
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