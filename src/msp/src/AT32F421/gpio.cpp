#include "at32f421_conf.h"
#include "gpio.h"
#include <assert.h>

GpioIf *GpioIf::new_instance(Pin pin)
{
    gpio_init_type gpio_init_struct;

    uint32_t pin_bit = GPIO2BIT(pin);
    gpio_type *regs = (gpio_type *)(GPIOA_BASE + 0x400 * GPIO2PORT(pin));
    Gpio::clock_enable(regs);
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_pins = pin_bit;

    gpio_init(regs, &gpio_init_struct);
    Gpio *gpio = new Gpio(regs, pin);
    return gpio;
}

void Gpio::clock_enable(gpio_type *regs)
{
    switch ((uint32_t)regs)
    {
    case GPIOA_BASE:
        crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
        break;
    case GPIOB_BASE:
        crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
        break;
    case GPIOC_BASE:
        crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
        break;
    case GPIOF_BASE:
        crm_periph_clock_enable(CRM_GPIOF_PERIPH_CLOCK, TRUE);
        break;

    default:
        assert(false);
        break;
    }
}

void Gpio::setup_af(Pin pin, AfMode mode, gpio_mux_sel_type function)
{
    uint32_t pin_idx = GPIO2IDX(pin);
    gpio_type *regs = (gpio_type *)(GPIOA_BASE + 0x400 * GPIO2PORT(pin));
    gpio_mode_type gpio_mode = GPIO_MODE_MUX;
    gpio_output_type gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_pull_type gpio_pull = GPIO_PULL_NONE;
    gpio_drive_type gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    Gpio::clock_enable(regs);
    switch (mode)
    {
    case AF_ANALOG:
        gpio_mode = GPIO_MODE_ANALOG;
        break;
    case AF_INPUT_FT:
        gpio_pull = GPIO_PULL_NONE;
    case AF_INPUT_PU:
        gpio_pull = GPIO_PULL_UP;
        break;
    case AF_INPUT_PD:
        gpio_pull = GPIO_PULL_DOWN;
        break;
    case AF_OUTPUT_PP:
        gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
        break;
    case AF_OUTPUT_OD:
        gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
        break;
    default:
        assert(false);
        break;
    }

    regs->cfgr &= (uint32_t) ~(0x03 << (pin_idx * 2));
    regs->cfgr |= (uint32_t)(gpio_mode << (pin_idx * 2));
    regs->omode &= (uint32_t) ~(0x01 << (pin_idx));
    regs->omode |= (uint32_t)(gpio_out_type << (pin_idx));
    regs->odrvr &= (uint32_t) ~(0x03 << (pin_idx * 2));
    regs->odrvr |= (uint32_t)(gpio_drive_strength << (pin_idx * 2));
    regs->pull &= (uint32_t) ~(0x03 << (pin_idx * 2));
    regs->pull |= (uint32_t)(gpio_pull << (pin_idx * 2));

    gpio_pin_mux_config(regs, (gpio_pins_source_type)pin_idx, function);
}

void Gpio::write(uint8_t bit)
{
    if (bit)
        set();
    else
        unset();
}

uint8_t Gpio::read()
{
    return (!!(regs->idt & pin_bit));
}

void Gpio::set()
{
    regs->scr = pin_bit;
}

void Gpio::unset()
{
    regs->clr = pin_bit;
}

void Gpio::toggle()
{
    if (read())
        unset();
    else
        set();
}

int Gpio::set_mode(IoMode mode)
{
    gpio_mode_type gpio_mode = GPIO_MODE_INPUT;
    gpio_output_type gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_pull_type gpio_pull = GPIO_PULL_NONE;
    switch (mode)
    {
    case INPUT_FLOAT:
        gpio_pull = GPIO_PULL_NONE;
        break;
    case INPUT_PULLUP:
        gpio_pull = GPIO_PULL_UP;
        break;
    case INPUT_PULLDOWN:
        gpio_pull = GPIO_PULL_DOWN;
        break;
    case OUTPUT:
        gpio_mode = GPIO_MODE_OUTPUT;
        gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
        break;
    case INPUT_OUTPUT:
        gpio_mode = GPIO_MODE_OUTPUT;
        gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
        break;
    default:
        return false;
    }

    regs->cfgr &= (uint32_t) ~(0x03 << (pin_idx * 2));
    regs->cfgr |= (uint32_t)(gpio_mode << (pin_idx * 2));
    regs->omode &= (uint32_t) ~(0x01 << (pin_idx));
    regs->omode |= (uint32_t)(gpio_out_type << (pin_idx));
    regs->pull &= (uint32_t) ~(0x03 << (pin_idx * 2));
    regs->pull |= (uint32_t)(gpio_pull << (pin_idx * 2));
    return 0;
}
