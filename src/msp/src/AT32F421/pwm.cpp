#include "pwm.h"
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

PwmMap pwm_maps[] = {
    // each pin must be different from each other.
    {TMR1, PA8, TMR_SELECT_CHANNEL_1, GPIO_MUX_2, DMA1_CHANNEL2},
    {TMR1, PA9, TMR_SELECT_CHANNEL_2, GPIO_MUX_2, DMA1_CHANNEL3},
    {TMR1, PA10, TMR_SELECT_CHANNEL_3, GPIO_MUX_2, DMA1_CHANNEL5},

    {TMR3, PB4, TMR_SELECT_CHANNEL_1, GPIO_MUX_0, DMA1_CHANNEL4},
    {TMR3, PB5, TMR_SELECT_CHANNEL_2, GPIO_MUX_0, 0},
    {TMR3, PB0, TMR_SELECT_CHANNEL_3, GPIO_MUX_1, DMA1_CHANNEL2},
    {TMR3, PB1, TMR_SELECT_CHANNEL_4, GPIO_MUX_1, DMA1_CHANNEL3},
    {TMR3, PA6, TMR_SELECT_CHANNEL_1, GPIO_MUX_1, DMA1_CHANNEL4},
    {TMR3, PA7, TMR_SELECT_CHANNEL_2, GPIO_MUX_1, 0},

    {TMR15, PA2, TMR_SELECT_CHANNEL_1, GPIO_MUX_0, DMA1_CHANNEL5},
    {TMR15, PA3, TMR_SELECT_CHANNEL_2, GPIO_MUX_0, DMA1_CHANNEL5},
};

PwmIf *PwmIf::new_instance(Pin pin)
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

    Pwm *new_pwm = new Pwm(timer, channel, dma);
    return new_pwm;
}

inline void Pwm::tim_config(tmr_type *tim, tmr_channel_select_type ch)
{
    uint32_t ch_idx = ch >> 1;
    // map the timer registers
    tim_cctrl = &tim->cctrl;
    tim_cdt = &(&tim->c1dt)[ch_idx];
    tim_pr = &tim->pr;
    tim_div = &tim->div;
    tim_cval = &tim->cval;
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

    tmr_output_enable(tim, TRUE);
    tmr_counter_enable(tim, TRUE);
}

inline void Pwm::dma_config(dma_channel_type *dma)
{
    // dma config
    dma_ctrl = nullptr;
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

Pwm::Pwm(tmr_type *tim, tmr_channel_select_type ch, dma_channel_type *dma)
{
    freq = 0;
    mode = OUTPUT; // OUTPUT default

    tim_config(tim, ch);
    dma_config(dma);

    set_freq(1000);
    set_dutycycle(0);
}

Pwm::~Pwm()
{
    // release dma
    if (dma_ctrl)
    {
        *dma_ctrl = 0;
        *dma_paddr = 0;
        *dma_maddr = 0;
        *dma_dtcnt = 0;
    }
    // release tim
    *tim_iden &= ~enable_dma_request;
    *tim_cctrl &= cctrl_mask;
    *tim_div = 0;
}

// dutycycle 0.0000 ~ 1.0
void Pwm::set_dutycycle(float dutycycle)
{
    if (__builtin_expect(this->dutycycle == dutycycle, false))
        return;
    this->dutycycle = dutycycle;
    *tim_cdt = dutycycle * cycle;
}

void Pwm::set_freq(uint32_t freq)
{
    if (__builtin_expect(this->freq == freq, false)) // the current min and max freq are 1000 and 12MHz, value outside that is illegal
        return;
    this->freq = freq;
    div = 1;
    cycle = SYS_CLOCK_FREQ / freq;
    while (__builtin_expect(cycle > 65535, false))
    {                // at most loop 3 times while freq = 1
        div <<= 4;   // *16
        cycle >>= 4; // /16
    }
    *tim_div = div - 1;    // attention: the div value will only take effect in the next cycle, so there is some delay according to the cycle length
    *tim_pr = cycle - 1;
    *tim_cdt = dutycycle * cycle;
}

uint32_t Pwm::get_duty() const
{
    return *tim_cdt;
}

uint32_t Pwm::get_cycle() const
{
    return cycle;
}

uint32_t Pwm::get_pos() const
{
    return *tim_cval;
}

void Pwm::enable()
{
    uint32_t tmp = *tim_cm;
    tmp &= coctrl_mask;
    tmp |= pwm_enable;
    *tim_cm = tmp;
}

void Pwm::disable()
{ // when pwm is disabled, the output pin must bo forced low
    uint32_t tmp = *tim_cm;
    tmp &= coctrl_mask;
    tmp |= pwm_disable;
    *tim_cm = tmp;
}

void Pwm::switch2output() // about 1us
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

    *tim_pr = cycle - 1;
    *tim_cctrl |= cctrl_out_value;

    if (dma_ctrl)
        *dma_ctrl = dma_m2p;
}

void Pwm::switch2input()
{
    *tim_cctrl &= cctrl_mask;

    uint32_t tmp = *tim_cm;
    tmp &= io_dir_mask;
    tmp |= pwm_input;
    *tim_cm = tmp;

    *tim_pr = 65535; // maximize the measuring range
    *tim_cctrl |= cctrl_in_value;

    if (dma_ctrl)
        *dma_ctrl = dma_p2m;
}

int Pwm::send_pulses(const uint32_t *pulses, uint32_t sz)
{
    assert(sz < BUF_SIZE);
    if (mode != OUTPUT)
    {
        mode = OUTPUT;
        switch2output();
    }

    if (!pulses)
        return *dma_dtcnt;

    uint32_t i = 0;
    uint32_t scale = 1000 * div;
    for (; i < sz; i++) // cost about 3us
    {
        uint32_t ticks = pulses[i] * (SYS_CLOCK_FREQ / 1000000) / scale; // map the pulse time to ticks
        buf[i] = ticks;
    }
    buf[i] = 0;
    *dma_ctrl &= ~1; // disable dma channel
    *dma_dtcnt = sz + 1;
    *dma_ctrl |= 1; // enable dma channel
    return 0;
}

#pragma GCC push_options
#pragma GCC optimize("O0")
inline void Pwm::restart_dma()
{
    if (!dma_ctrl)
        return;
    register uint32_t tmp = *dma_ctrl;
    register uint32_t disable_dma = tmp & ~1; // unset dma enable bit
    register uint32_t enable_dma = tmp | 1;   // set dma enable bit
    *dma_ctrl = disable_dma;
    *dma_dtcnt = rd_sz + 1;
    *dma_ctrl = enable_dma;
    *dma_ctrl = disable_dma; // first restart to clear the last cached DMA event, which is not expected, it may be at32's bug
    *dma_dtcnt = rd_sz + 1;
    *dma_ctrl = enable_dma;
}
#pragma GCC pop_options

int Pwm::recv_pulses(uint32_t *pulses, uint32_t sz)
{ // support max frame length 500us, which is far more enough for dshot150 to dshot 1200
    assert(sz < BUF_SIZE);

    if (mode != INPUT)
    {
        mode = INPUT;
        switch2input();
    }

    if (__builtin_expect(pulses == nullptr, true))
    {
        if (__builtin_expect(!user_buf, 0))
            return 0;
        int data_sz = rd_sz - (int)*dma_dtcnt;
        if (rd_idx >= data_sz)
            return rd_idx; // no new data

        uint32_t scale = 256 * 1000 * div / (SYS_CLOCK_FREQ / 1000000); // count of ns in one tick, multiplied by 256 to lower the dividing error;
        for (; rd_idx < data_sz; rd_idx++)
        {
            register uint16_t last_tick = buf[rd_idx];
            register uint16_t now_tick = buf[rd_idx + 1];
            uint32_t delta_tick;
            if (now_tick < last_tick)
                delta_tick = now_tick + cycle - last_tick;
            else
                delta_tick = now_tick - last_tick;
            user_buf[rd_idx] = delta_tick * scale / 256;
        }
        return rd_idx;
    }
    rd_idx = 0;
    user_buf = pulses;
    rd_sz = sz;
    restart_dma();
    return 0;
}

uint32_t high_pulse;

int Pwm::recv_high_pulse()
{
    return 0;
}

void Pwm::set_polarity(int edge)
{
    if (edge > 0)
        cctrl_out_value = cctrl_out_high_value;
    else
        cctrl_out_value = cctrl_out_low_value;
}

void TMR3_GLOBAL_IRQHandler(void)
{
    extern uint32_t pulse;
    static uint8_t bit = 0;
    static uint32_t start = 0;
    uint32_t end;
    TMR3->ists = 0;
    if (bit == 0)
        start = TMR3->c1dt;
    else
    {
        end = TMR3->c1dt;
        if (end < start)
            pulse = TMR3->pr + 1 - start + end;
        else
            pulse = end - start;
    }
    bit = !bit;
    TMR3->cctrl_bit.c1p = bit;
}
