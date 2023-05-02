#ifndef __PIDcontrol_H_
#define __PIDcontrol_H__

#include <stdint.h>
#include <Adafruit_MCP4725.h>

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

class PID_control
{

  int16_t x_desired;		// desired output
  int16_t x_actual[2];		// actual output

  float e;			// Previous error value
  float u;			// Previous control input 

  const unsigned long sampleTime;	// Sampling interval
  unsigned long previousMicros;	// will store last time LED was updated

  const float Kp;		// Proportional gain
  const float Ki_times_DeltaT;		// Integral gain
  const float Kd_over_DeltaT;		// Derivative gain
  const float eMax;             // Slew rate limit

  Adafruit_MCP4725* dac;
  Encoder* encoder;

  const int lockLED;
  const int alarmLED;
  const int indexPin;

  unsigned lockTimer;

public:
    PID_control (Adafruit_MCP4725* _dac,
                 Encoder* _encoder, int encoder_IDX,
                 int lockLEDpin, int alarmLEDpin,
		 unsigned long _sampleTime, float _Kp, float _Ti, float _Td);

  void begin (); 

  void setSystemoutput (int16_t value);
  uint16_t getSysteminput ();
  int16_t getSystemoutput ();

  void update (unsigned long currentMicros);
};

#endif
