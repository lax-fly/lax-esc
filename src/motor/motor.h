#pragma once
#include <stdint.h>

#include "config.h"

#define TONE1 1600
#define TONE2 1800
#define TONE3 2000
#define TONE4 2200
#define TONE5 2400
#define TONE6 2600

class MotorIf
{
public:
    enum Type
    {
        BDCM, // brushed DC motor
        BLDC, // brushless DC motor
        PMSM, // Permanent magnet synchronous motor(in future)
    };
    enum VolumeLevel
    {
        VOLUME_OFF, // OFF
        VOLUME_LOW,
        VOLUME_MID,
        VOLUME_HIGH,
    };
    virtual int get_rpm() const = 0; // the sign of return value represents the dir, negative means backward
    virtual int get_erpm() const = 0;
    virtual int get_e_period() const = 0;
    virtual int get_current() const = 0;    // return current consumption in mA, return -1 if function isn't supported
#ifndef NDEBUG
    virtual int get_throttle() const = 0;   // get the throttle set by user
    virtual int get_real_throttle() const = 0;   // get the real throttle currently running at(for debug)
#endif
    virtual void set_throttle(int v) = 0;   // suport negative throttle as backward spin direction, or just treat negative throttle as 0 if not supported
    virtual void stop() = 0;
    virtual void beep(uint32_t freq, VolumeLevel volume) = 0; // setting freq to 0 means beep off
    virtual void arm(bool state) = 0;
    virtual bool is_armed() = 0;
    virtual void poll() = 0;
    static MotorIf *singleton(Type type);
};
