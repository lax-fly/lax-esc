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

public:
    Sound(MotorIf *motor) : motor(motor) { timer = TimerIf::singleton(); }
    void power_on_tone(void);
    void armed_tone(void);
    void throttle_signal_detected_tone(void);
};