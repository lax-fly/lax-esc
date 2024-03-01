#pragma once
#include "msp.h"

class Bldc : public MotorIf
{
private:

public:
    Bldc();
    ~Bldc();
    virtual int get_rpm() const;
    virtual int get_erpm() const;
    virtual int get_e_period() const;
    virtual int get_current() const;
    virtual void set_throttle(float v);
    virtual void stop();
    virtual void beep(uint32_t freq, VolumeLevel volume);
    virtual void poll();
};