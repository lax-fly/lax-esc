#pragma once

#include <stdint.h>
#include "msp.h"
#include "at32f421_tmr.h"

class MotorPwm : public MotorPwmIf
{
private:
    uint32_t dutycycle; // 0-2000 map to 0.0-1.0
    uint32_t freq;
    uint16_t duty;
    uint32_t cycle;

    Pin pina, pinb, pinc;

    uint32_t ch1_cm1;
    uint32_t ch1_cm2;
    uint32_t ch2_cm1;
    uint32_t ch2_cm2;
    uint32_t ch3_cm1;
    uint32_t ch3_cm2;

    volatile uint32_t *tim_cdt1;
    volatile uint32_t *tim_cdt2;
    volatile uint32_t *tim_cdt3;
    volatile uint32_t *tim_cdt;
    volatile uint32_t *tim_pr;
    volatile uint32_t *tim_div;
    volatile uint32_t *tim_cm1;
    volatile uint32_t *tim_cm2;
    volatile uint32_t *tim_cval;

    inline void tim_config(tmr_type *tim, tmr_channel_select_type ch1, tmr_channel_select_type ch2, tmr_channel_select_type ch3);

public:
    MotorPwm(Pin a, Pin b, Pin c);
    ~MotorPwm();
    virtual void set_dutycycle(uint32_t dutycycle);
    virtual void set_freq(uint32_t freq);
    virtual void select(Pin pin);
    virtual uint32_t get_duty() const;  // return the pwm duty length
    virtual uint32_t get_cycle() const; // return the pwm cycle length
    virtual uint32_t get_pos() const;   // return the current pwm output position in the cycle
};
