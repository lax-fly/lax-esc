#include "signal-pwm.h"
#include "gpio.h"
#include "assert.h"

#define ARRAY_CNT(x) (sizeof(x) / sizeof(x[0]))
#define BUF_SIZE 64
#define SYS_CLOCK_FREQ 120000000

#define DEFAULT_PWM_PERIOD 65536

struct PwmMap
{
    tmr_type *periph;
    uint16_t pin;
    tmr_channel_select_type ch;
    gpio_mux_sel_type af;
    dma_channel_type *dma;
};

static PwmMap pwm_maps[] = {
    // each pin must be different from each other.
    {TMR1, PA8, TMR_SELECT_CHANNEL_1, GPIO_MUX_2, DMA1_CHANNEL2},
    {TMR1, PA9, TMR_SELECT_CHANNEL_2, GPIO_MUX_2, DMA1_CHANNEL3},
    {TMR1, PA10, TMR_SELECT_CHANNEL_3, GPIO_MUX_2, DMA1_CHANNEL5},

    {TMR3, PB4, TMR_SELECT_CHANNEL_1, GPIO_MUX_0, DMA1_CHANNEL4},
    {TMR3, PB0, TMR_SELECT_CHANNEL_3, GPIO_MUX_1, DMA1_CHANNEL2},
    {TMR3, PB1, TMR_SELECT_CHANNEL_4, GPIO_MUX_1, DMA1_CHANNEL3},
    {TMR3, PA6, TMR_SELECT_CHANNEL_1, GPIO_MUX_1, DMA1_CHANNEL4},

    {TMR15, PA2, TMR_SELECT_CHANNEL_1, GPIO_MUX_0, DMA1_CHANNEL5},
    {TMR15, PA3, TMR_SELECT_CHANNEL_2, GPIO_MUX_0, DMA1_CHANNEL5},
};

SignalPwmIf *SignalPwmIf::new_instance(Pin pin)
{
    SignalPwm *new_pwm = new SignalPwm(pin);
    return new_pwm;
}

inline void SignalPwm::tim_config(tmr_type *tim, tmr_channel_select_type ch)
{
    uint32_t ch_idx = ch >> 1;
    // map the timer registers
    tim_cctrl = &tim->cctrl;
    tim_cdt = &(&tim->c1dt)[ch_idx];
    tim_pr = &tim->pr;
    tim_div = &tim->div;
    tim_iden = &tim->iden;

    enable_dma_request = 1 << (9 + ch_idx);
    cctrl_mask = ~(0b1111 << (4 * ch_idx));
    if (ch_idx < 2)
    {
        tim_cm = &tim->cm1;
        io_dir_mask = ~(0xff << (8 * ch_idx));
        coctrl_mask = ~(0b111 << (4 + 8 * ch_idx));
        pwm_enable = TMR_OUTPUT_CONTROL_PWM_MODE_A << (4 + 8 * ch_idx);
        pwm_disable = TMR_OUTPUT_CONTROL_FORCE_LOW << (4 + 8 * ch_idx);
    }
    else
    {
        tim_cm = &tim->cm2;
        io_dir_mask = ~(0xff << (8 * (ch_idx - 2)));
        coctrl_mask = ~(0b111 << (4 + 8 * (ch_idx - 2)));
        pwm_enable = TMR_OUTPUT_CONTROL_PWM_MODE_A << (4 + 8 * (ch_idx - 2));
        pwm_disable = TMR_OUTPUT_CONTROL_FORCE_LOW << (4 + 8 * (ch_idx - 2));
    }

    switch ((uint32_t)tim)
    {
    case TMR1_BASE:
        crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
        break;
    case TMR3_BASE:
        crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
        break;
    case TMR15_BASE:
        crm_periph_clock_enable(CRM_TMR15_PERIPH_CLOCK, TRUE);
        break;
    default:
        assert(false);
        break;
    }

    tmr_base_init(tim, DEFAULT_PWM_PERIOD - 1, 0);
    tmr_cnt_dir_set(tim, TMR_COUNT_UP);
    tmr_clock_source_div_set(tim, TMR_CLOCK_DIV1);
    tmr_period_buffer_enable(tim, FALSE);

    tmr_sub_sync_mode_set(tim, FALSE);
    tmr_primary_mode_select(tim, TMR_PRIMARY_SEL_RESET);
    tmr_overflow_request_source_set(tim, FALSE);

    tmr_input_config_type tmr_input_struct;
    tmr_input_struct.input_channel_select = ch;
    tmr_input_struct.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
    tmr_input_struct.input_polarity_select = TMR_INPUT_BOTH_EDGE;
    tmr_input_struct.input_filter_value = 0;
    tmr_input_channel_init(tim, &tmr_input_struct, TMR_CHANNEL_INPUT_DIV_1);
    cctrl_in_value = *tim_cctrl & ~cctrl_mask;
    pwm_input = *tim_cm & ~io_dir_mask;
    cctrl_in_high_value = ~0b1010 & cctrl_in_value;
    cctrl_in_low_value = ~0b1000 & cctrl_in_value;
    cctrl_in_both_value = cctrl_in_value;

    tmr_output_config_type tmr_output_struct;
    tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_output_struct.oc_output_state = TRUE;
    tmr_output_struct.occ_output_state = FALSE;
    tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.oc_idle_state = FALSE;
    tmr_output_struct.occ_idle_state = FALSE;
    tmr_output_channel_config(tim, ch, &tmr_output_struct);
    tmr_channel_value_set(tim, ch, 0);
    tmr_output_channel_buffer_enable(tim, ch, TRUE);
    cctrl_out_high_value = *tim_cctrl & ~cctrl_mask;
    cctrl_out_low_value = cctrl_out_high_value | (TMR_OUTPUT_ACTIVE_LOW << (ch_idx * 4 + 1));
    cctrl_out_value = cctrl_out_high_value;
    pwm_output = *tim_cm & ~io_dir_mask;
    it_enable = TMR_C1_INT << ch_idx;
    it_disable = ~it_enable;

    tmr_output_enable(tim, TRUE);
    tmr_counter_enable(tim, TRUE);
    nvic_irq_enable(TMR3_GLOBAL_IRQn, 0, 0);
}

inline void SignalPwm::dma_release()
{
    if (dma_ctrl)
    {
        *dma_ctrl = 0;
        *dma_paddr = 0;
        *dma_maddr = 0;
        *dma_dtcnt = 0;
        dma_ctrl = 0;
        *tim_iden &= ~enable_dma_request; // enable tmr's dma request
    }
}

inline void SignalPwm::dma_config(dma_channel_type *dma)
{
    // dma config
    if (!dma || dma->paddr) // check if dma is occupied by checking the peripheral address
        return;

    dma_ctrl = &dma->ctrl;
    dma_paddr = &dma->paddr;
    dma_maddr = &dma->maddr;
    dma_dtcnt = &dma->dtcnt;

    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    dma->ctrl &= 0xbfee;
    dma->ctrl |= DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma->ctrl_bit.chpl = DMA_PRIORITY_LOW;
    dma->ctrl_bit.mwidth = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma->ctrl_bit.pwidth = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma->ctrl_bit.mincm = TRUE;
    dma->ctrl_bit.pincm = FALSE;
    dma->ctrl_bit.lm = FALSE;
    *dma_paddr = (uint32_t)tim_cdt;
    *dma_maddr = (uint32_t)buf;
    dma_m2p = *dma_ctrl;
    dma_p2m = (dma_m2p & 0xbfee) | DMA_DIR_PERIPHERAL_TO_MEMORY;
    *tim_iden |= enable_dma_request; // enable tmr's dma request
}

SignalPwm::SignalPwm(Pin pin)
{
    uint8_t index = 0;
    for (index = 0; index < ARRAY_CNT(pwm_maps); index++)
    {
        if (pin == pwm_maps[index].pin)
            break;
    }

    assert(index < ARRAY_CNT(pwm_maps));

    struct PwmMap &pwm_map = pwm_maps[index];

    Gpio::setup_af(pin, Gpio::AF_OUTPUT_PP, pwm_map.af);

    tmr_type *timer = pwm_map.periph;
    tmr_channel_select_type channel = pwm_map.ch;
    dma_channel_type *dma = pwm_map.dma;

    dma_ctrl = nullptr;
    callback = nullptr;

    tim_config(timer, channel);
    dma_config(dma);
    set_mode(PULSE_OUTPUT_CAPTURE);
}

SignalPwm::~SignalPwm()
{
    // release dma
    dma_release();
    // release tim
    *tim_iden &= ~enable_dma_request;
    *tim_cctrl &= cctrl_mask;
    *tim_div = 0;
}

void SignalPwm::switch2output() // about 1us
{
    *tim_cctrl &= cctrl_mask;
    // *tim_cdt = 0; // will be loaded at the start of next period for we enabled channel output data buffer, so the next code line is necessary
    // *tim_pr = 1;  // force to overflow to make the channel reg load tim_cdt(0) in one clock to avoid a unexpected pulse

    uint32_t tmp = *tim_cm;
    tmp &= io_dir_mask;
    *tim_cm = tmp & ~(1 << 3); // disable cdt output buffer
    *tim_cdt = 0;              // set tim cdt immediately when output buffer is disabled
    tmp |= pwm_output;
    *tim_cm = tmp; // enable output buffer

    // *tim_pr = cycle - 1;
    *tim_cctrl |= cctrl_out_value;

    if (dma_ctrl)
        *dma_ctrl = dma_m2p;
}

void SignalPwm::switch2input()
{
    *tim_cctrl &= cctrl_mask;

    uint32_t tmp = *tim_cm;
    tmp &= io_dir_mask;
    tmp |= pwm_input;
    *tim_cm = tmp;

    *tim_cctrl |= cctrl_in_value;

    if (dma_ctrl)
        *dma_ctrl = dma_p2m;
}

int SignalPwm::send_pulses(const uint16_t *pulses, uint32_t sz, uint32_t period)
{
    assert(sz < BUF_SIZE);
    if (io_dir != OUTPUT)
    {
        io_dir = OUTPUT;
        switch2output();
    }

    if (!pulses)
        return *dma_dtcnt;

    *tim_pr = period - 1;
    ((uint16_t *)pulses)[sz] = 0;
    *dma_ctrl &= ~1; // disable dma channel
    *dma_maddr = (uint32_t)pulses;
    *dma_dtcnt = sz + 1;
    *dma_ctrl |= 1; // enable dma channel
    return 0;
}

#pragma GCC push_options
#pragma GCC optimize("O0")
inline void SignalPwm::restart_dma(uint16_t *buf, uint32_t sz)
{
    if (!dma_ctrl)
        return;
    register uint32_t tmp = *dma_ctrl;
    register uint32_t disable_dma = tmp & ~1; // unset dma enable bit
    register uint32_t enable_dma = tmp | 1;   // set dma enable bit
    *dma_ctrl = disable_dma;
    *dma_dtcnt = sz;
    *dma_maddr = (uint32_t)buf;
    *dma_ctrl = enable_dma;
    *dma_ctrl = disable_dma; // first restart to clear the last cached DMA event, which is not expected, it may be at32's bug
    *dma_dtcnt = sz;
    *dma_ctrl = enable_dma;
}
#pragma GCC pop_options

int SignalPwm::recv_pulses(uint16_t *pulses, uint32_t sz)
{ // support max frame length 500us, which is far more enough for dshot150 to dshot 1200
    assert(sz < BUF_SIZE);

    if (io_dir != INPUT)
    {
        io_dir = INPUT;
        switch2input();
    }

    if (__builtin_expect(pulses == nullptr, true))
    {
        return (int)*dma_dtcnt;
    }
    *tim_pr = DEFAULT_PWM_PERIOD - 1; // maximize the measuring range
    restart_dma(pulses, sz);
    return sz;
}

SignalPwm *pwm_tim3 = nullptr;
SignalPwmIf::Callback tim3_callback = nullptr;

void SignalPwm::set_mode(Mode mode)
{
    pwm_tim3 = nullptr;
    tim3_callback = nullptr;
    switch (mode)
    {
    case UP_PULSE_CAPTURE:
        *tim_div = 15; // attention: the div value will only take effect in the next cycle, so there is some delay according to the cycle length
        cctrl_in_value = cctrl_in_high_value;
        pwm_tim3 = this;
        tim3_callback = callback;
        *tim_iden = it_enable;
        break;
    case PULSE_OUTPUT_CAPTURE:
        *tim_div = 0; // attention: the div value will only take effect in the next cycle, so there is some delay according to the cycle length
        cctrl_in_value = cctrl_in_both_value;
        cctrl_out_value = cctrl_out_low_value;
        *tim_iden &= it_disable;
        break;
    default:
        break;
    }
    *tim_pr = DEFAULT_PWM_PERIOD - 1;
    io_dir = INPUT;
        
    switch2input();
}

void SignalPwm::set_up_pulse_callback(Callback cb)
{
    callback = cb;
    tim3_callback = callback;
}

volatile uint32_t &tim3_ists = TMR3->ists;
volatile uint32_t &tim3_cctrl = TMR3->cctrl;

void TMR3_GLOBAL_IRQHandler(void)
{
    static uint32_t start_tick = 0;
    tim3_ists = 0;
    if (tim3_cctrl == pwm_tim3->cctrl_in_high_value)
    {
        tim3_cctrl = pwm_tim3->cctrl_in_low_value;
        start_tick = *pwm_tim3->tim_cdt;
    }
    else
    {
        tim3_cctrl = pwm_tim3->cctrl_in_high_value;
        if (tim3_callback)
        {
            uint32_t tick = *pwm_tim3->tim_cdt;
            if (tick < start_tick)
                tick = tick + DEFAULT_PWM_PERIOD - start_tick;
            else
                tick -= start_tick;

            uint32_t scale = 256 * 1000 * 16 / (SYS_CLOCK_FREQ / 1000000); // count of ns in one tick, multiplied by 256 to lower the dividing error;
            uint32_t pulse = tick * scale / 256;
            tim3_callback(pulse);
        }
    }
}
