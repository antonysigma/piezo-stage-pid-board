#include <Wire.h>

#include <Adafruit_MCP4725.h>

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

//#define __PID_INVERTED_OUTPUT__
#include "PIDcontrol.h"
#include "config.h"

namespace {
Adafruit_MCP4725 dac;
Encoder myEnc(encoder_A, encoder_B);

// Update PID every 5 millisecond
PID_control PID(&dac, &myEnc);

volatile int16_t position;
auto buffer = (unsigned char*) &position;
volatile bool newPosition = false;

void testTriangleSignal(const unsigned long currentTime) {
  static unsigned long previousTime = currentTime;
  static int16_t counter = 0;

  // Monitor values every 1 second
  if (currentTime - previousTime < 3000000L)
    return;

  PID.setSystemoutput(counter);

  // Step change of 50 micrometer
  // Range = -40 to 260
  if(counter >= 250)
    counter = -50;
  else 
    counter += 50;

  previousTime = currentTime;
}

template<typename T>
constexpr T
clamp(const T x, const T vmin, const T vmax) {
  if (x > vmax) {
    return vmax;
  }

  if (x < vmin) {
    return vmin;
  }

  return x;
}
}

// TODO(Antony): Linker asked for the missing delete symbol. Not sure where the
// malloc is implemented in the 3rd party code.
void operator delete(void* ptr, unsigned int) {
  free(ptr);
}

void receiveEvent(int numBytes)
{
  buffer[0] = Wire.read(); // receive byte as a character
  buffer[1] = Wire.read();
  newPosition = true;
}

void setup(void) {
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

  //PID.setSystemoutput (100);
  
}

void loop() {
  unsigned long currentTime = micros();

  PID.update(currentTime);

  if(newPosition)
  {
    Serial.println(position);
    PID.setSystemoutput (clamp(position, -50, 250));  
    newPosition = false;
  }

}


