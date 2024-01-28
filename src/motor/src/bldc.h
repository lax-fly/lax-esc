#pragma once
#include "msp.h"

typedef struct
{
    void (*commutate)(void);
    int step_fall;
    int step_rise;
    int step_stall;
    ComparatorIf *cmp;
    AdcIf *adc;
    PwmIf *pwm;
} CommutateMap;

class Bldc : public MotorIf
{
private:

public:
    Bldc();
    virtual uint32_t get_rpm() const;
    virtual uint32_t get_current() const;
    virtual void set_throttle(float v);
    virtual int set_direction(int dir);
    virtual void arm(bool state);
    virtual void stop();
    virtual void beep(uint32_t freq, VolumeLevel volume);
    virtual void poll();
};