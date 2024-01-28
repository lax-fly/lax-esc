#pragma once

#include <stdint.h>
#include "msp.h"
#include "at32f421_gpio.h"

class Gpio : public GpioIf
{
private:
    gpio_type * regs;
    uint16_t pin_idx;
    uint16_t pin_bit;

public:
    enum AfMode
    {
        AF_ANALOG,
        AF_INPUT_FT,
        AF_INPUT_PU,
        AF_INPUT_PD,
        AF_OUTPUT_PP,
        AF_OUTPUT_OD
    };
    static void clock_enable(gpio_type* regs);
    virtual void write(uint8_t bit);
    virtual uint8_t read();
    virtual void set();
    virtual void unset();
    virtual void toggle();
    virtual int set_mode(IoMode mode);
    Gpio(gpio_type * regs, Pin pin) : regs(regs), pin_idx(GPIO2IDX(pin)), pin_bit(GPIO2BIT(pin))
    {
    }
    static void setup_af(Pin pin, AfMode mode, gpio_mux_sel_type function = GPIO_MUX_0);
};

