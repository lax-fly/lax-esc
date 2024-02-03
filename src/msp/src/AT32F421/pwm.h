#pragma once

#include <stdint.h>
#include "msp.h"
#include "at32f421_tmr.h"

class Pwm : public PwmIf
{
private:
    tmr_type *tim;
    tmr_channel_select_type ch;
    uint8_t ch_idx;
    dma_channel_type *dma;
    uint32_t *ch_dr;
    float dutycycle;
    uint32_t freq;
    uint16_t duty;
    uint16_t cycle;
    uint16_t* tx_buf;
    uint16_t* rx_buf;
    uint32_t rd_idx;
    uint32_t *user_buf;
    uint32_t user_buf_sz;
    uint32_t reg_out_cctrl;
    uint32_t reg_in_cctrl;
    uint32_t reg_in_cm;
    uint32_t reg_out_cm;
    uint32_t mode;
    void dma_config();
    void switch2output();
    void switch2input();

public:
    Pwm(tmr_type *tim, tmr_channel_select_type ch, dma_channel_type *dma);
    virtual void set_dutycycle(float dutycycle);
    virtual void set_freq(uint32_t freq);
    virtual uint32_t get_duty() const;  // return the pwm duty length
    virtual uint32_t get_cycle() const; // return the pwm cycle length
    virtual uint32_t get_pos() const;   // return the current pwm output position in the cycle
    virtual void enable();
    virtual void disable();
    virtual void set_mode(Mode mode);
    virtual int serial_write(const uint32_t *pulses, uint32_t sz); // async
    virtual int serial_read(uint32_t *data, uint32_t sz);          // async
};
