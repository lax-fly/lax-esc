#pragma once

#include <stdint.h>
#include "msp.h"
#include "at32f421_tmr.h"

extern "C" void TMR3_GLOBAL_IRQHandler(void);
extern "C" void TMR1_GLOBAL_IRQHandler(void);
extern "C" void TMR15_GLOBAL_IRQHandler(void);

class Pwm : public PwmIf
{
private:
    enum
    {
        INPUT = 0,  // to recieve pwm input
        OUTPUT = 1, // to output pwm, polarity is low(idle high)
    }io_dir;

    float dutycycle;
    uint32_t freq;
    uint16_t duty;
    uint32_t cycle;

    uint16_t buf[64];
    int rd_idx;
    uint32_t *user_buf;
    int rd_sz;

    dma_channel_type *dma;
    uint32_t dma_m2p;
    uint32_t dma_p2m;
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
    uint32_t cctrl_out_high_value;
    uint32_t cctrl_out_low_value;
    uint32_t cctrl_out_value;
    uint32_t cctrl_in_value;
    uint32_t it_enable;
    uint32_t it_disable;
    uint32_t cctrl_in_high_value;
    uint32_t cctrl_in_low_value;
    uint32_t cctrl_in_both_value;
    volatile uint32_t *tim_cdt;
    volatile uint32_t *tim_cctrl;
    volatile uint32_t *tim_cm;
    volatile uint32_t *tim_pr;
    volatile uint32_t *tim_div;
    volatile uint32_t *tim_cval;
    volatile uint32_t *tim_iden;

    Callback callback;

    void switch2output();
    void switch2input();
    inline void tim_config(tmr_type *tim, tmr_channel_select_type ch);
    inline void restart_dma();
    inline void dma_config();
    inline void dma_release();

    friend void TMR3_GLOBAL_IRQHandler(void);
    friend void TMR1_GLOBAL_IRQHandler(void);
    friend void TMR15_GLOBAL_IRQHandler(void);

public:
    Pwm(tmr_type *tim, tmr_channel_select_type ch, dma_channel_type *dma);
    ~Pwm();
    virtual void set_mode(Mode mode = PWM_OUTPUT);
    virtual void set_dutycycle(float dutycycle);
    virtual void set_freq(uint32_t freq);
    virtual uint32_t get_duty() const;  // return the pwm duty length
    virtual uint32_t get_cycle() const; // return the pwm cycle length
    virtual uint32_t get_pos() const;   // return the current pwm output position in the cycle
    virtual void enable();
    virtual void disable();
    virtual int send_pulses(const uint32_t *pulses, uint32_t sz, uint32_t period); // async
    virtual int recv_pulses(uint32_t *pulses, uint32_t sz);
    virtual void set_up_pulse_callback(Callback cb);
};
