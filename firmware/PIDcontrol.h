#ifndef __PIDcontrol_H_
#define __PIDcontrol_H__

#include <Adafruit_MCP4725.h>
#include <Encoder.h>
#include <stdint.h>

class PID_control {
    // Option to invert the system output.
    static constexpr bool INVERT_OUTPUT = true;
    static constexpr auto sign = INVERT_OUTPUT ? -1 : 1;

    static constexpr unsigned long sampleTime = 500;  // Sampling interval
    static constexpr float Kp = 2e1f * sign;          // Proportional gain
    static constexpr float Ti = 1e-2f;                // Integral gain
    static constexpr float Td = 5e-4f;                // Derivative gain

    // Slew rate limiter: limit changes to 50um / 5ms = 20 count / ms
    static constexpr float slewRatelimit = 20e3f;

    static constexpr float Ki_times_DeltaT = Kp / Ti * 1e-6f * sampleTime;
    static constexpr float Kd_over_DeltaT = Kp * Td * 1e6f / sampleTime;
    static constexpr float eMax = slewRatelimit * sampleTime * 1e-6f;  // Slew rate limit
    static constexpr uint16_t systemInputdefault = 3277;  // 4096U * 4 / 5;

    int16_t x_desired = 0;    // desired output
    int16_t x_actual[2] = {0, 0};  // actual output

    float e = 0;  // Previous error value
    float u = systemInputdefault;  // Previous control input

    unsigned long previousMicros;  // will store last time LED was updated

    Adafruit_MCP4725* dac;
    Encoder* encoder;

    unsigned lockTimer;

   public:
    PID_control(Adafruit_MCP4725* _dac, Encoder* _encoder);

    void begin();

    void setSystemoutput(int16_t value);
    uint16_t getSysteminput() const;
    int16_t getSystemoutput() const;

    void update(unsigned long currentMicros);
};

#endif
