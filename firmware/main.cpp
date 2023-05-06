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
volatile bool newPosition = false;

void
testTriangleSignal(const unsigned long currentTime, PID_control& PID) {
    static unsigned long previousTime = currentTime;
    static int16_t counter = 0;

    // Monitor values every 1 second
    if (currentTime - previousTime < 3000000L) return;

    PID.setSystemoutput(counter);

    // Step change of 50 micrometer
    // Range = -40 to 260
    if (counter >= 250)
        counter = -50;
    else
        counter += 50;

    previousTime = currentTime;
}

void
receiveEvent(int numBytes) {
    auto buffer = reinterpret_cast<volatile unsigned char*>(&position);
    buffer[0] = Wire.read();  // receive byte as a character
    buffer[1] = Wire.read();
    newPosition = true;
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
    Encoder myEnc(encoder_A, encoder_B);

    // Update PID every 5 millisecond
    PID_control PID(&dac, &myEnc);

    Serial.begin(115200);
    Serial.println(F("Hello!"));

    // Register the MCU in multi-master mode, having I2C address 0x09.
    Wire.begin(MCU_ADDR);

    // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
    // For MCP4725A0 the address is 0x60 or 0x61
    // For MCP4725A2 the address is 0x64 or 0x65
    dac.begin(DAC_ADDR, &Wire);
    Wire.onReceive(receiveEvent);

    // TODO: Find zero position
    Serial.println(F("Resetting encoder"));
    PID.begin();

    for (;;) {
        unsigned long currentTime = micros();

        PID.update(currentTime);

        if (newPosition) {
            using utils::clamp;
            // Serial.println(position);
            PID.setSystemoutput(clamp(position, -50, 250));
            newPosition = false;
        }
    }
}