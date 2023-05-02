/**************************************************************************/
/*! 
 @file     trianglewave.pde
 @author   Adafruit Industries
 @license  BSD (see license.txt)
 
 This example will generate a triangle wave with the MCP4725 DAC.   
 
 This is an example sketch for the Adafruit MCP4725 breakout board
 ----> http://www.adafruit.com/products/935
 
 Adafruit invests time and resources providing this open source code, 
 please support Adafruit and open-source hardware by purchasing 
 products from Adafruit!
 */
/**************************************************************************/
#include <Wire.h>

#include <Adafruit_MCP4725.h>

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#if defined(ENCODER_USE_INTERRUPTS) && defined(ENCODER_OPTIMIZE_INTERRUPTS)
#if defined(__AVR__)
#if defined(INT0_vect) && CORE_NUM_INTERRUPT > 0
ISR(INT0_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(0)]); }
#endif
#if defined(INT1_vect) && CORE_NUM_INTERRUPT > 1
ISR(INT1_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(1)]); }
#endif
#if defined(INT2_vect) && CORE_NUM_INTERRUPT > 2
ISR(INT2_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(2)]); }
#endif
#if defined(INT3_vect) && CORE_NUM_INTERRUPT > 3
ISR(INT3_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(3)]); }
#endif
#if defined(INT4_vect) && CORE_NUM_INTERRUPT > 4
ISR(INT4_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(4)]); }
#endif
#if defined(INT5_vect) && CORE_NUM_INTERRUPT > 5
ISR(INT5_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(5)]); }
#endif
#if defined(INT6_vect) && CORE_NUM_INTERRUPT > 6
ISR(INT6_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(6)]); }
#endif
#if defined(INT7_vect) && CORE_NUM_INTERRUPT > 7
ISR(INT7_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(7)]); }
#endif
#endif // AVR
#if defined(attachInterrupt)
// Don't intefere with other libraries or sketch use of attachInterrupt()
// https://github.com/PaulStoffregen/Encoder/issues/8
#undef attachInterrupt
#endif
#endif // ENCODER_OPTIMIZE_INTERRUPTS

//#define __PID_INVERTED_OUTPUT__
#include "PIDcontrol.h"

/********************************************************************************/

enum {
encoder_A = 2,
encoder_B = 3,
encoder_IDX = 4,
lockLED = 6,
alarmLED = 7
};

Adafruit_MCP4725 dac;
Encoder myEnc(encoder_A, encoder_B);

// Update PID every 5 millisecond
PID_control PID(&dac, &myEnc, encoder_IDX, lockLED, alarmLED, 500, -2e1f, 1e-2f, 5e-4f);

volatile int16_t position;
unsigned char *buffer = (unsigned char*) &position;
volatile bool newPosition = false;

void receiveEvent(int numBytes)
{
  buffer[0] = Wire.read(); // receive byte as a character
  buffer[1] = Wire.read();
  newPosition = true;
}

const uint16_t dac_offset = 3277; //4096L / 5Volt * 4Volt

void setup(void) {
  Serial.begin(115200);
  Serial.println("Hello!");

  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  dac.begin(0x63, 0x09);
  Wire.onReceive(receiveEvent);

  // TODO: Find zero position
  Serial.println("Resetting encoder");
  PID.begin();

  //PID.setSystemoutput (100);
  
}

void loop() {
  //static unsigned long previousTime = micros();
  //static int16_t counter = 0;
  //int16_t output;

  unsigned long currentTime = micros();

  PID.update(currentTime);

  // Monitor values every 1 second
  //if (currentTime - previousTime >= 3000000L)
  //{
  //PID.setSystemoutput(counter);

  // Step change of 50 micrometer
  // Range = -40 to 260
  //if(counter >= 250)
  //  counter = -50;
  //else 
  //  counter += 50;

  //previousTime = currentTime;
  //}

  if(newPosition)
  {
    Serial.println(position);
    if (position > 250)
      position = 250;
    else if (position < -50)
      position = -50;

    PID.setSystemoutput (position);  
    newPosition = false;
  }

}


