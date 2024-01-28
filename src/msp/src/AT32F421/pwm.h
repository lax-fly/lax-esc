#pragma once

#include <stdint.h>
#include "msp.h"
#include "at32f421_tmr.h"

#define DEFAULT_PWM_PERIOD 10000

class Pwm : public PwmIf
{
private:
    tmr_type *m_tim;
    tmr_channel_select_type m_ch;
    dma_channel_type* m_dma;
    uint32_t* m_ch_dr;
    float m_dutycycle;
    uint32_t m_freq;
    uint16_t m_duty;
    uint16_t m_cycle;
    uint16_t m_tx_buf[64];
    void dma_config();

public:
    Pwm(tmr_type *tim, tmr_channel_select_type ch, dma_channel_type* dma);
    virtual void set_dutycycle(float dutycycle);
    virtual void set_freq(uint32_t freq);
    virtual uint32_t duty() const;  // return the pwm duty length
    virtual uint32_t cycle() const; // return the pwm cycle length
    virtual uint32_t pos() const;   // return the current pwm output position in the cycle
    virtual void enable();
    virtual void disable();
    virtual void set_mode(Mode mode);
    virtual int serial_write(const uint32_t *pulses, uint32_t sz); // async
    virtual void read_digital(uint8_t *data, uint32_t sz);  // async
};
