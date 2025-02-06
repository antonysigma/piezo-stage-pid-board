#pragma once

#include <wiring_private.h>

#include "callbacks.hpp"

namespace components {
namespace core {
static constexpr auto timer0_init = flow::action("TimerInit"_sc, []() {
    sei();

#if defined(__AVR_ATmega128__)
    // CPU specific: different values for the ATmega128
    sbi(TCCR0, CS02);
#elif defined(TCCR0) && defined(CS01) && defined(CS00)
        // this combination is for the standard atmega8
        sbi(TCCR0, CS01);
        sbi(TCCR0, CS00);
#elif defined(TCCR0B) && defined(CS01) && defined(CS00)
        // this combination is for the standard 168/328/1280/2560
        sbi(TCCR0B, CS01);
        sbi(TCCR0B, CS00);
#elif defined(TCCR0A) && defined(CS01) && defined(CS00)
        // this combination is for the __AVR_ATmega645__ series
        sbi(TCCR0A, CS01);
        sbi(TCCR0A, CS00);
#else
#error Timer 0 prescale factor 64 not set correctly
#endif
});

static constexpr auto disable_usart = flow::action("DisableUSART"_sc, []() {
#if defined(UCSRB)
    UCSRB = 0;
#elif defined(UCSR0B)
        UCSR0B = 0;
#endif
});

struct impl {
    constexpr static auto config = cib::config(cib::extend<RuntimeInit>(  //
        timer0_init >> disable_usart));
};

}  // namespace core
}  // namespace components