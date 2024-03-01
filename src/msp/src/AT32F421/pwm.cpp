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

void Pwm::dma_config()
{
    if (using_dma)
        return;
    using_dma = true;
    // dma config
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    dma_init_type dma_init_struct;
    dma_reset(dma);
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_LOW;
    dma_init_struct.loop_mode_enable = FALSE;
    dma_init(dma, &dma_init_struct);
    dma->ctrl_bit.chen = 0; // disable channel
    dma->paddr = (uint32_t)ch_dr;

    dma->maddr = (uint32_t)buf;
    tim->iden |= 1 << (9 + ch_idx); // enable tmr's dma request
}

Pwm::Pwm(tmr_type *tim, tmr_channel_select_type ch, dma_channel_type *dma)
{

    tmr_output_config_type tmr_output_struct;

    switch ((uint32_t)tim)
    {
    case TMR1_BASE:
        crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
        break;
    case TMR3_BASE:
        crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
        break;
    case TMR14_BASE:
        crm_periph_clock_enable(CRM_TMR14_PERIPH_CLOCK, TRUE);
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
    reg_out_cctrl = tim->cctrl & (0b1111 << 4 * (ch / 2));
    if (ch < TMR_SELECT_CHANNEL_3)
        reg_out_cm = tim->cm1 & (0xff << 8 * (ch / 2));
    else
        reg_out_cm = tim->cm2 & (0xff << 8 * (ch / 2 - 2));

    tmr_input_config_type tmr_input_struct;
    tmr_input_struct.input_channel_select = ch;
    tmr_input_struct.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
    tmr_input_struct.input_polarity_select = TMR_INPUT_BOTH_EDGE;
    tmr_input_struct.input_filter_value = 0;
    tmr_input_channel_init(tim, &tmr_input_struct, TMR_CHANNEL_INPUT_DIV_1);
    reg_in_cctrl = tim->cctrl & (0b1111 << 4 * (ch / 2));
    if (ch < TMR_SELECT_CHANNEL_3)
        reg_in_cm = tim->cm1 & (0xff << 8 * (ch / 2));
    else
        reg_in_cm = tim->cm2 & (0xff << 8 * (ch / 2 - 2));

    tmr_output_enable(tim, TRUE);
    tmr_counter_enable(tim, TRUE);

    this->dma = dma;
    this->tim = tim;
    this->ch = ch;
    this->ch_idx = ch >> 1;
    this->ch_dr = &(&tim->c1dt)[ch_idx];
    this->freq = 0;
    this->using_dma = false;
    set_freq(1000);
    set_dutycycle(0);

    switch2output(); // OUTPUT default
}

Pwm::~Pwm()
{
}

// dutycycle 0.0000 ~ 1.0
void Pwm::set_dutycycle(float dutycycle)
{
    if (__builtin_expect(this->dutycycle == dutycycle, false))
        return;
    this->dutycycle = dutycycle;
    *ch_dr = dutycycle * cycle;
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
    tim->div = div - 1;
    tim->pr = cycle - 1;
    *ch_dr = dutycycle * cycle;
}

uint32_t Pwm::get_duty() const
{
    return *ch_dr;
}

uint32_t Pwm::get_cycle() const
{
    return cycle;
}

uint32_t Pwm::get_pos() const
{
    return tim->cval;
}

void Pwm::enable()
{
    switch (ch)
    {
    case TMR_SELECT_CHANNEL_1:
        tim->cm1_output_bit.c1octrl = TMR_OUTPUT_CONTROL_PWM_MODE_A;
        break;
    case TMR_SELECT_CHANNEL_2:
        tim->cm1_output_bit.c2octrl = TMR_OUTPUT_CONTROL_PWM_MODE_A;
        break;
    case TMR_SELECT_CHANNEL_3:
        tim->cm2_output_bit.c3octrl = TMR_OUTPUT_CONTROL_PWM_MODE_A;
        break;
    case TMR_SELECT_CHANNEL_4:
        tim->cm2_output_bit.c4octrl = TMR_OUTPUT_CONTROL_PWM_MODE_A;
        break;
    default:
        assert(false);
        break;
    }
}

void Pwm::disable()
{ // when pwm is disabled, the output pin must bo forced low
    switch (ch)
    {
    case TMR_SELECT_CHANNEL_1:
        tim->cm1_output_bit.c1octrl = TMR_OUTPUT_CONTROL_FORCE_LOW;
        break;
    case TMR_SELECT_CHANNEL_2:
        tim->cm1_output_bit.c2octrl = TMR_OUTPUT_CONTROL_FORCE_LOW;
        break;
    case TMR_SELECT_CHANNEL_3:
        tim->cm2_output_bit.c3octrl = TMR_OUTPUT_CONTROL_FORCE_LOW;
        break;
    case TMR_SELECT_CHANNEL_4:
        tim->cm2_output_bit.c4octrl = TMR_OUTPUT_CONTROL_FORCE_LOW;
        break;
    default:
        assert(false);
        break;
    }
}

void Pwm::switch2output() // about 1us
{
    tim->cctrl &= ~(0b1111 << 4 * ch_idx);
    *ch_dr = 0;  // will be loaded at the start of next period for we enabled channel output data buffer, so the next code line is necessary
    tim->pr = 1; // force to overflow to make the channel reg load ch_dr(0) in one clock to avoid a unexpected pulse

    if (ch < TMR_SELECT_CHANNEL_3)
    {
        uint32_t cm = tim->cm2;
        cm &= ~(0xff << 8 * ch_idx);
        cm |= reg_out_cm;
        tim->cm1 = cm;
    }
    else
    {
        uint32_t cm = tim->cm2;
        cm &= ~(0xff << 8 * (ch_idx - 2));
        cm |= reg_out_cm;
        tim->cm2 = cm;
    }
    tim->pr = cycle - 1;
    tim->cctrl |= reg_out_cctrl;

    // uint32_t tmp = dma->ctrl;
    // tmp &= 0xbfee; // lsb is channel enable bit, disable it first
    // tmp |= DMA_DIR_MEMORY_TO_PERIPHERAL;
    // dma->ctrl = tmp;
}

void Pwm::switch2input()
{
    tim->cctrl &= ~(0b1111 << 4 * ch_idx);
    tim->pr = 65535;
    if (ch < TMR_SELECT_CHANNEL_3)
    {
        uint32_t cm = tim->cm1;
        cm &= ~(0xff << 8 * ch_idx);
        cm |= reg_in_cm;
        tim->cm1 = cm;
    }
    else
    {
        uint32_t cm = tim->cm2;
        cm &= ~(0xff << 8 * (ch_idx - 2));
        cm |= reg_in_cm;
        tim->cm2 = cm;
    }
    tim->cctrl |= reg_in_cctrl;

    // uint32_t tmp = dma->ctrl;
    // tmp &= 0xbfee;
    // // tmp |= DMA_DIR_PERIPHERAL_TO_MEMORY; // no need for DMA_DIR_PERIPHERAL_TO_MEMORY = 0
    // dma->ctrl = tmp;
    // user_buf = nullptr;
}

void Pwm::set_mode(PwmIf::Mode mode)
{
    switch (mode)
    {
    case PwmIf::INPUT:
    {
        dma_config();
        switch2input();
        break;
    }
    case PwmIf::OUTPUT:
    {
        dma_config();
        switch2output();
        break;
    }
    default:
        assert(false);
        break;
    }
}

int Pwm::send_pulses(const uint32_t *pulses, uint32_t sz)
{
    assert(sz < BUF_SIZE);
    if (!pulses)
        return dma->dtcnt;

    uint32_t i = 0;
    uint32_t scale = 1000 * div;
    for (; i < sz; i++) // cost about 3us
    {
        uint32_t ticks = pulses[i] * (SYS_CLOCK_FREQ / 1000000) / scale; // map the pulse time to ticks
        buf[i] = ticks;
    }
    buf[i] = 0;
    dma->ctrl_bit.chen = 0;
    dma->dtcnt = sz + 1;
    dma->ctrl_bit.chen = 1;
    return 0;
}

#pragma GCC push_options
#pragma GCC optimize("O0")
static inline void restart_dma(dma_channel_type *dma, uint32_t sz)
{
    volatile uint32_t &ctrl = dma->ctrl;
    volatile uint32_t &dtcnt = dma->dtcnt;
    register uint32_t tmp = ctrl;
    register uint32_t disable_dma = tmp & ~1; // unset dma enable bit
    register uint32_t enable_dma = tmp | 1;   // set dma enable bit
    ctrl = disable_dma;
    dtcnt = sz;
    ctrl = enable_dma;
    ctrl = disable_dma; // first restart to clear the last cached DMA event, which is not expected, it may be at32's bug
    dtcnt = sz;
    ctrl = enable_dma;
}
#pragma GCC pop_options

int Pwm::recv_pulses(uint32_t *pulses, uint32_t sz)
{ // support max frame length 500us, which is far more enough for dshot150 to dshot 1200
    assert(sz < BUF_SIZE);

    if (__builtin_expect(pulses == nullptr, true))
    {
        if (__builtin_expect(!user_buf, 0))
            return 0;
        int data_sz = rd_sz - (int)dma->dtcnt;
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
    restart_dma(dma, sz + 1);
    return 0;
}
