#include "callbacks.hpp"
#include "components/alarm.hpp"
#include "components/core.hpp"

namespace {
struct harness {
    static constexpr auto config = cib::config(cib::exports<RuntimeInit>, cib::exports<MainLoop>);
};

using namespace components;
struct project {
    static constexpr auto config = cib::components<  //
        harness, core::init,                         //
        alarm::init                                  //
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