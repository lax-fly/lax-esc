#pragma once

#include <stdint.h>
#include "msp.h"
#include "at32f421_tmr.h"

class Pwm : public PwmIf
{
private:
    float dutycycle;
    uint32_t freq;
    uint16_t duty;
    uint32_t cycle;
    uint32_t div;
    
    uint16_t buf[64];
    int rd_idx;
    uint32_t *user_buf;
    int rd_sz;

    volatile uint32_t *dma_ctrl;
    volatile uint32_t *dma_paddr;
    volatile uint32_t *dma_maddr;
    volatile uint32_t *dma_dtcnt;

    uint32_t pwm_enable;
    uint32_t pwm_disable;
    uint32_t pwm_input;
    uint32_t pwm_output;
    uint32_t io_dir_mask;
    uint32_t cctrl_mask;
    uint32_t coctrl_mask;
    uint32_t enable_dma_request;
    uint32_t cctrl_out_value;
    uint32_t cctrl_in_value;
    volatile uint32_t *tim_cdt;
    volatile uint32_t *tim_cctrl;
    volatile uint32_t *tim_cm;
    volatile uint32_t *tim_pr;
    volatile uint32_t *tim_div;
    volatile uint32_t *tim_cval;
    volatile uint32_t *tim_iden;

    Mode mode;
    void dma_config();
    void switch2output();
    void switch2input();
    inline void restart_dma();

public:
    Pwm(tmr_type *tim, tmr_channel_select_type ch, dma_channel_type *dma);
    ~Pwm();
    virtual void set_dutycycle(float dutycycle);
    virtual void set_freq(uint32_t freq);
    virtual uint32_t get_duty() const;  // return the pwm duty length
    virtual uint32_t get_cycle() const; // return the pwm cycle length
    virtual uint32_t get_pos() const;   // return the current pwm output position in the cycle
    virtual void enable();
    virtual void disable();
    virtual void set_mode(Mode mode);
    virtual int send_pulses(const uint32_t *pulses, uint32_t sz); // async
    virtual int recv_pulses(uint32_t *pulses, uint32_t sz);
    virtual int recv_high_pulse();
};
