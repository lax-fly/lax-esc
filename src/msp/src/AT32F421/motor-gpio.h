#pragma once

#include <stdint.h>
#include "msp.h"
#include "at32f421_gpio.h"

class MotorIo : public MotorIoIf
{
private:
    uint16_t pin_a;
    uint16_t pin_b;
    uint16_t pin_c;

    uint16_t pin_a_bit;
    uint16_t pin_b_bit;
    uint16_t pin_c_bit;

    volatile uint32_t* set_a;
    volatile uint32_t* set_b;
    volatile uint32_t* set_c;
    volatile uint32_t* clr_a;
    volatile uint32_t* clr_b;
    volatile uint32_t* clr_c;

public:
    virtual void select(Pin pin);
    MotorIo(Pin a, Pin b, Pin c);
};

