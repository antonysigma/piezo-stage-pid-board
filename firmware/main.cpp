#include "boost/sml.hpp"

// Include sml.hpp first
#include <Adafruit_MCP4725.h>
#include <Wire.h>

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#include "PIDcontrol.h"
#include "config.h"
#include "utils.hpp"

namespace {
volatile int16_t position = 0;
volatile bool new_position = false;

/** Helper function to decode the fields in the SRAM as byte stream. */
class bytes_t {
   public:
    using value_type = volatile uint8_t;
    template <typename T>
    constexpr bytes_t(volatile T& field)
        : data{reinterpret_cast<value_type*>(&field)}, size{sizeof(T)} {}

    constexpr value_type* begin() const { return data; }

    constexpr value_type* end() const { return data + size; }

   private:
    value_type* data;
    const size_t size;
};

void
testTriangleSignal(const uint32_t current_time, PIDControl* pid) {
    static uint32_t previous_time = current_time;
    static int16_t counter = 0;

    // Monitor values every 1 second
    if (current_time - previous_time < 3000000L) return;

    pid->setSystemoutput(counter);

    // Step change of 50 micrometer
    // Range = -40 to 260
    if (counter >= 250) {
        counter = -50;
    } else {
        counter += 50;
    }

    previous_time = current_time;
}

void
receiveEvent(int numBytes) {
    for (auto& c : bytes_t{position}) {
        c = Wire.read();
    }

    new_position = true;
}

}  // namespace

// TODO(Antony): Linker asked for the missing delete symbol. Not sure where the
// malloc is implemented in the 3rd party code.
void
operator delete(void* ptr, unsigned int) {
    free(ptr);
}

int
main() {
    Adafruit_MCP4725 dac;
    Encoder my_enc(encoder_A, encoder_B);

    // Update PID every 5 millisecond
    PIDControl pid(&dac, &my_enc);

    Serial.begin(115200);
    Serial.println(F("Hello!"));

    // Register the MCU in multi-master mode, having I2C address 0x09.
    Wire.begin(MCU_ADDR);

    // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
    // For MCP4725A0 the address is 0x60 or 0x61
    // For MCP4725A2 the address is 0x64 or 0x65
    dac.begin(DAC_ADDR, &Wire);
    Wire.onReceive(receiveEvent);

    // TODO(Antony): Find zero position
    Serial.println(F("Resetting encoder"));
    pid.begin();

    for (;;) {
        const uint32_t current_time = micros();

        pid.update(current_time);

        if (new_position) {
            using utils::clamp;
            // Serial.println(position);
            pid.setSystemoutput(clamp(position, -50, 250));
            new_position = false;
        }
    }
}