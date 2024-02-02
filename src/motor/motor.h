#pragma once
#include <stdint.h>

class MotorIf
{
public:
    enum Type
    {
        BDCM,   // brushed DC motor
        BLDC,   // brushless DC motor
        PMSM,   // Permanent magnet synchronous motor(in future)
    };
    enum VolumeLevel
    {
        VOLUME_OFF, // OFF
        VOLUME_LOW,
        VOLUME_MID,
        VOLUME_HIGH,
    };
    virtual int get_rpm() const = 0;            // the sign of return value represents the dir, negative means backward
    virtual uint32_t get_current() const = 0;   // return current consumption in mA
    virtual void set_throttle(float v) = 0;     // suport negative throttle as backward spin direction, or just treat negative throttle as 0 if not supported
    virtual void arm(bool state) = 0;
    virtual void stop() = 0;
    virtual void beep(uint32_t freq, VolumeLevel volume) = 0;   // setting freq to 0 means beep off
    virtual void poll() = 0;
    static MotorIf* singleton(Type type);
};
