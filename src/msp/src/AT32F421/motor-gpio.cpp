#include "at32f421_conf.h"
#include "gpio.h"
#include "motor-gpio.h"
#include <assert.h>

MotorIoIf *MotorIoIf::new_instance(Pin a, Pin b, Pin c)
{
    MotorIo *gpio = new MotorIo(a, b, c);
    return gpio;
}

MotorIo::MotorIo(Pin a, Pin b, Pin c)
{
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;

    pin_a_bit = GPIO2BIT(a);
    pin_b_bit = GPIO2BIT(b);
    pin_c_bit = GPIO2BIT(c);

    gpio_type *regsa = (gpio_type *)(GPIOA_BASE + 0x400 * GPIO2PORT(a));
    gpio_type *regsb = (gpio_type *)(GPIOA_BASE + 0x400 * GPIO2PORT(b));
    gpio_type *regsc = (gpio_type *)(GPIOA_BASE + 0x400 * GPIO2PORT(c));
    Gpio::clock_enable(regsa);
    Gpio::clock_enable(regsb);
    Gpio::clock_enable(regsc);

    gpio_init_struct.gpio_pins = pin_a_bit;
    gpio_init(regsa, &gpio_init_struct);
    gpio_init_struct.gpio_pins = pin_b_bit;
    gpio_init(regsb, &gpio_init_struct);
    gpio_init_struct.gpio_pins = pin_c_bit;
    gpio_init(regsc, &gpio_init_struct);

    set_a = &regsa->scr;
    set_b = &regsb->scr;
    set_c = &regsc->scr;

    clr_a = &regsa->clr;
    clr_b = &regsb->clr;
    clr_c = &regsc->clr;

    pin_a = a;
    pin_b = b;
    pin_c = c;
}

void MotorIo::select(Pin pin)
{
    if (pin == pin_a)
    {
        *set_a = pin_a_bit;
        *clr_b = pin_b_bit;
        *clr_c = pin_c_bit;
    }else if (pin == pin_b)
    {
        *clr_a = pin_a_bit;
        *set_b = pin_b_bit;
        *clr_c = pin_c_bit;
    }else if (pin == pin_c)
    {
        *clr_a = pin_a_bit;
        *clr_b = pin_b_bit;
        *set_c = pin_c_bit;
    }
    else if (pin == PIN_MAX)
    {
        *set_a = pin_a_bit;
        *set_b = pin_b_bit;
        *set_c = pin_c_bit;
    }
    else
    {
        *clr_a = pin_a_bit;
        *clr_b = pin_b_bit;
        *clr_c = pin_c_bit;
    }
}
