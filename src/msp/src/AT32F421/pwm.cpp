#include "pwm.h"
#include "gpio.h"
#include "assert.h"

#define ARRAY_CNT(x) (sizeof(x) / sizeof(x[0]))
#define BUF_SIZE 64
#define TIM_FREQ_DIV 2
#define TIM_FREQ (120000000 / TIM_FREQ_DIV)

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

    {TMR14, PA4, TMR_SELECT_CHANNEL_1, GPIO_MUX_0, 0},

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

    struct PwmMap pwm_map = pwm_maps[index];

    Gpio::setup_af(pin, Gpio::AF_OUTPUT_PP, pwm_map.af);

    tmr_type *timer = pwm_map.periph;
    tmr_channel_select_type channel = pwm_map.ch;
    dma_channel_type *dma = pwm_map.dma;

    Pwm *new_pwm = new Pwm(timer, channel, dma);
    return new_pwm;
}

void Pwm::dma_config()
{
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
    dma->paddr = (uint32_t)ch_dr;
    dma->ctrl_bit.chen = 0;         // disable channel
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

    tmr_base_init(tim, DEFAULT_PWM_PERIOD - 1, TIM_FREQ_DIV - 1);
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
    this->dutycycle = 0;
    this->duty = 0;
    this->cycle = DEFAULT_PWM_PERIOD - 1;
    this->freq = TIM_FREQ / this->cycle;

    this->tx_buf = nullptr;
    this->rx_buf = nullptr;

    this->mode = OUTPUT;
    switch2output(); // OUTPUT default
}

// dutycycle 0.0000 ~ 1.0
void Pwm::set_dutycycle(float dutycycle)
{
    if (this->dutycycle == dutycycle)
        return;
    this->dutycycle = dutycycle;
    duty = dutycycle * cycle;
    *ch_dr = duty;
}

void Pwm::set_freq(uint32_t freq)
{
    if (this->freq == freq || freq < 1000 || freq > 12000000) // the current min and max freq are 1000 and 12MHz, value outside that is illegal
        return;
    this->freq = freq;
    cycle = TIM_FREQ / freq; // tim's clock frequency
    duty = dutycycle * cycle;
    tim->pr = cycle - 1;
    *ch_dr = duty;
}

uint32_t Pwm::get_duty() const
{
    return *ch_dr;
}

uint32_t Pwm::get_cycle() const
{
    return tim->pr + 1;
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

    if (mode & SERIAL)
    {
        uint32_t tmp = dma->ctrl;
        tmp &= 0xbfee; // lsb is channel enable bit, disable it first
    tmp |= DMA_DIR_MEMORY_TO_PERIPHERAL;
        dma->ctrl = tmp;
    }
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

    if (mode & SERIAL)
    {
        uint32_t tmp = dma->ctrl;
        tmp &= 0xbfee;
        // tmp |= DMA_DIR_PERIPHERAL_TO_MEMORY; // no need for DMA_DIR_PERIPHERAL_TO_MEMORY = 0
        dma->ctrl = tmp;
        user_buf = nullptr;
    }
}

void Pwm::set_mode(PwmIf::Mode mode)
{
    this->mode |= mode;
    switch (mode)
    {
    case PwmIf::INPUT:
    {
        switch2input();
        break;
    }
    case PwmIf::OUTPUT:
    {
        switch2output();
        break;
    }
    case PwmIf::SERIAL:
        dma_config();
        rx_buf = new uint16_t[BUF_SIZE];
        tx_buf = new uint16_t[BUF_SIZE];
        break;
    default:
        assert(false);
        break;
    }
}

int Pwm::serial_write(const uint32_t *pulses, uint32_t sz)
{
    assert(sz < BUF_SIZE);
    if (!pulses)
        return dma->dtcnt;

    uint32_t i = 0;
    uint32_t period = 1000000000 / TIM_FREQ; // make sure TIM_FREQ <= 1000000000
    for (; i < sz; i++)                      // cost 3us
    {
        assert(pulses[i] < 50000); // support only pulse below 50us
        uint32_t pulse = pulses[i] / period;
        tx_buf[i] = pulse;
    }
    tx_buf[i] = 0;
    tim->cval = 0;
    dma->ctrl_bit.chen = 0;
    dma->dtcnt = sz + 1;
    dma->maddr = (uint32_t)tx_buf;
    dma->ctrl_bit.chen = 1;
    return 0;
}

#pragma GCC push_options
#pragma GCC optimize("O0")
static inline void restart_dma(dma_channel_type *dma, uint16_t *buf, uint32_t sz)
{
    register uint32_t tmp = dma->ctrl;
    register uint32_t disable_dma = tmp & ~1; // unset dma eable bit
    register uint32_t enable_dma = tmp | 1;   // set dma eable bit
    dma->ctrl = disable_dma;
    dma->dtcnt = sz;
    dma->maddr = (uint32_t)buf;
    dma->ctrl = enable_dma;
    dma->ctrl = disable_dma; // first restart to clear the last cached DMA event, which is not expected, it may be at32's bug
    dma->dtcnt = sz;
    dma->ctrl = enable_dma;
}
#pragma GCC pop_options

int Pwm::serial_read(uint32_t *data, uint32_t sz)
{ // support max frame length 500us, which is far more enough for dshot150 to dshot 1200
    assert(sz < BUF_SIZE);
    if (!data)
    {
        if (__builtin_expect(!user_buf, 0))
            return rd_idx;
        uint32_t data_sz = user_buf_sz + 1 - dma->dtcnt;
        if (rd_idx + 1 >= data_sz)
            return rd_idx; // no new data

        uint32_t scale = 256 * 1000 / (TIM_FREQ / 1000000); // count of ns in one tick, multiplied by 256 to lower the dividing error;
        for (; rd_idx < data_sz - 1; rd_idx++)
        {
            register uint16_t last_tick = rx_buf[rd_idx];
            register uint16_t now_tick = rx_buf[rd_idx + 1];
            uint32_t delta_tick;
            if (now_tick < last_tick)
                delta_tick = now_tick + 65536 - last_tick;
            else
                delta_tick = now_tick - last_tick;
            user_buf[rd_idx] = delta_tick * scale / 256;
        }
        return rd_idx;
    }
    rd_idx = 0;
    user_buf = data;
    user_buf_sz = sz;
    tim->cval = 0;
    // tim->ists = 0;
    restart_dma(dma, rx_buf, sz + 1);
    return 0;
}
