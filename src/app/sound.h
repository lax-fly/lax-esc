#pragma once

#include "motor.h"
#include "msp.h"

extern TimerIf *timer;
extern MotorIf *motor;

class Sound
{
private:
    TimerIf *timer;
    MotorIf *motor;
    int state;
    uint32_t run_time;
    uint32_t freq;
    MotorIf::VolumeLevel volume;
    uint32_t duration;

public:
    Sound(MotorIf *motor) : motor(motor), state(3), run_time(0) { timer = TimerIf::singleton(); }
    void power_on_tone(void);
    void armed_tone(void);
    void throttle_signal_detected_tone(void);
    void poll(void);
};