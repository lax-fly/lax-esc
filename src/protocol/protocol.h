#pragma once
#include "msp.h"

class Protocol
{
public:
    enum Type
    {
        AUTO_DETECT,
        BRUSHED,
        STD_PWM,
        ONESHOT,    // oneshot125 oneshot42 multishot
        DSHOT,      // dshot 150 300 600,bi-direction dshot
        PROSHOT,
        SERIAL,
    };
    virtual ~Protocol() {}
    // the callback should copy the package(including the str), then process it outside the callback, warning: don't make the callback take too long
    virtual void poll(void) = 0;
    static Protocol *singleton(Type type, Pin pin);
    static Type auto_detect(Pin pin);
};