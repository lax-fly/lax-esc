#pragma once

#include <stdint.h>
#include "msp.h"
#include "at32f421_tmr.h"

extern "C" void TMR3_GLOBAL_IRQHandler(void);
extern "C" void TMR1_GLOBAL_IRQHandler(void);
extern "C" void TMR15_GLOBAL_IRQHandler(void);

class SignalPwm : public SignalPwmIf
{
private:
    enum
    {
        INPUT = 0,  // to recieve pwm input
        OUTPUT = 1, // to output pwm, polarity is low(idle high)
    }io_dir;

    uint16_t buf[64];

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
    uint32_t cctrl_in_high_value;
    uint32_t cctrl_in_low_value;
    uint32_t cctrl_in_both_value;
    volatile uint32_t *tim_cdt;
    volatile uint32_t *tim_cctrl;
    volatile uint32_t *tim_cm;
    volatile uint32_t *tim_pr;
    volatile uint32_t *tim_div;
    volatile uint32_t *tim_iden;

    Callback callback;

    void switch2output();
    void switch2input();
    inline void tim_config(tmr_type *tim, tmr_channel_select_type ch);
    inline void restart_dma(uint16_t* buf, uint32_t sz);
    inline void dma_config(dma_channel_type *dma);
    inline void dma_release();

    friend void TMR3_GLOBAL_IRQHandler(void);
    friend void TMR1_GLOBAL_IRQHandler(void);
    friend void TMR15_GLOBAL_IRQHandler(void);

public:
    SignalPwm(Pin pin);
    ~SignalPwm();
    virtual void set_mode(Mode mode);
    virtual int send_pulses(const uint16_t *pulses, uint32_t sz, uint32_t period); // async
    virtual int recv_pulses(uint16_t *pulses, uint32_t sz);
    virtual void set_up_pulse_callback(Callback cb);
};
