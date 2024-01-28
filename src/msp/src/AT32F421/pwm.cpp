#include "pwm.h"
#include "gpio.h"
#include "assert.h"

#define ARRAY_CNT(x) (sizeof(x) / sizeof(x[0]))

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

    tmr_output_config_type tmr_output_struct;

    switch ((uint32_t)timer)
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

    tmr_base_init(timer, DEFAULT_PWM_PERIOD - 1, 7);
    tmr_cnt_dir_set(timer, TMR_COUNT_UP);
    tmr_clock_source_div_set(timer, TMR_CLOCK_DIV1);
    tmr_period_buffer_enable(timer, FALSE);

    tmr_sub_sync_mode_set(timer, FALSE);
    tmr_primary_mode_select(timer, TMR_PRIMARY_SEL_RESET);
    tmr_overflow_request_source_set(timer, TRUE);

    tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_output_struct.oc_output_state = TRUE;
    tmr_output_struct.occ_output_state = FALSE;
    tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.oc_idle_state = FALSE;
    tmr_output_struct.occ_idle_state = FALSE;
    tmr_output_channel_config(timer, channel, &tmr_output_struct);
    tmr_channel_value_set(timer, channel, 0);
    tmr_output_channel_buffer_enable(timer, channel, FALSE);

    tmr_output_enable(timer, TRUE);
    tmr_counter_enable(timer, TRUE);

    Pwm *new_pwm = new Pwm(timer, channel, dma);
    return new_pwm;
}

void Pwm::dma_config()
{
    // dma config
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    dma_init_type dma_init_struct;
    dma_reset(m_dma);
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_LOW;
    dma_init_struct.loop_mode_enable = FALSE;
    dma_init(m_dma, &dma_init_struct);
    m_dma->paddr = (uint32_t)m_ch_dr;
    m_dma->ctrl_bit.chen = 0; // disable channel
    m_tim->iden |= TMR_OVERFLOW_DMA_REQUEST;// enable tmr's dma request
}

Pwm::Pwm(tmr_type *tim, tmr_channel_select_type ch, dma_channel_type *dma) : m_tim(tim),
                                                                             m_ch(ch),
                                                                             m_dutycycle(0),
                                                                             m_freq(0),
                                                                             m_duty(0),
                                                                             m_cycle(DEFAULT_PWM_PERIOD)
{
    uint32_t ch_idx = m_ch >> 1;
    m_ch_dr = &(&m_tim->c1dt)[ch_idx];
    m_dma = dma;
}

// m_dutycycle 0.0000 ~ 1.0
void Pwm::set_dutycycle(float dutycycle)
{
    if (this->m_dutycycle == dutycycle)
        return;
    m_dutycycle = dutycycle;
    m_duty = m_dutycycle * m_cycle;
    *m_ch_dr = m_duty;
}

void Pwm::set_freq(uint32_t freq)
{
    if (this->m_freq == freq || freq < 300 || freq > 15000)   // the current min and max freq are 300 and 15000, value outside that is illegal
        return;
    m_freq = freq;
    m_cycle = (120000000 / 8) / m_freq; // tim's clock frequency
    m_duty = m_dutycycle * m_cycle;
    m_tim->pr = m_cycle - 1;
    *m_ch_dr = m_duty;
}

uint32_t Pwm::duty() const
{
    return *m_ch_dr;
}

uint32_t Pwm::cycle() const
{
    return m_tim->pr + 1;
}

uint32_t Pwm::pos() const
{
    return m_tim->cval;
}

void Pwm::enable()
{
    switch (m_ch)
    {
    case TMR_SELECT_CHANNEL_1:
        m_tim->cm1_output_bit.c1octrl = TMR_OUTPUT_CONTROL_PWM_MODE_A;
        break;
    case TMR_SELECT_CHANNEL_2:
        m_tim->cm1_output_bit.c2octrl = TMR_OUTPUT_CONTROL_PWM_MODE_A;
        break;
    case TMR_SELECT_CHANNEL_3:
        m_tim->cm2_output_bit.c3octrl = TMR_OUTPUT_CONTROL_PWM_MODE_A;
        break;
    case TMR_SELECT_CHANNEL_4:
        m_tim->cm2_output_bit.c4octrl = TMR_OUTPUT_CONTROL_PWM_MODE_A;
        break;
    default:
        assert(false);
        break;
    }
}

void Pwm::disable()
{ // when pwm is disabled, the output pin must bo forced low
    switch (m_ch)
    {
    case TMR_SELECT_CHANNEL_1:
        m_tim->cm1_output_bit.c1octrl = TMR_OUTPUT_CONTROL_FORCE_LOW;
        break;
    case TMR_SELECT_CHANNEL_2:
        m_tim->cm1_output_bit.c2octrl = TMR_OUTPUT_CONTROL_FORCE_LOW;
        break;
    case TMR_SELECT_CHANNEL_3:
        m_tim->cm2_output_bit.c3octrl = TMR_OUTPUT_CONTROL_FORCE_LOW;
        break;
    case TMR_SELECT_CHANNEL_4:
        m_tim->cm2_output_bit.c4octrl = TMR_OUTPUT_CONTROL_FORCE_LOW;
        break;
    default:
        assert(false);
        break;
    }
}

void Pwm::set_mode(PwmIf::Mode mode)
{
    switch (mode)
    {
    case PwmIf::INPUT:
    {
        register uint32_t tmp = m_dma->ctrl;
        tmp &= 0xbfef;
        tmp |= DMA_DIR_PERIPHERAL_TO_MEMORY;
        m_dma->ctrl = tmp;
        tmr_input_config_type tmr_input_struct;
        tmr_input_struct.input_channel_select = m_ch;
        tmr_input_struct.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
        tmr_input_struct.input_polarity_select = TMR_INPUT_RISING_EDGE;
        tmr_input_struct.input_filter_value = 0;
        tmr_input_channel_init(m_tim, &tmr_input_struct, TMR_CHANNEL_INPUT_DIV_1);
        break;
    }
    case PwmIf::OUTPUT:
    {
        register uint32_t tmp = m_dma->ctrl;
        tmp &= 0xbfef;
        tmp |= DMA_DIR_MEMORY_TO_PERIPHERAL;
        m_dma->ctrl = tmp;
        tmr_output_config_type tmr_output_struct;
        tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
        tmr_output_struct.oc_output_state = TRUE;
        tmr_output_struct.occ_output_state = FALSE;
        tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
        tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
        tmr_output_struct.oc_idle_state = FALSE;
        tmr_output_struct.occ_idle_state = FALSE;
        tmr_output_channel_config(m_tim, m_ch, &tmr_output_struct);
        break;
    }
    case PwmIf::DIGITAL:
        dma_config();
    default:
        assert(false);
        break;
    }
}

int Pwm::serial_write(const uint32_t *pulses, uint32_t sz)
{
    assert(sz < ARRAY_CNT(m_tx_buf));
    if (!pulses)
        return m_dma->dtcnt == 0;

    uint32_t i = 0;
    for (; i < sz; i++)
    {
        uint32_t pulse = (float)pulses[i] * m_cycle * m_freq / 1000000000;
        m_tx_buf[i] = pulse;
    }
    m_tx_buf[i] = 0;
    m_tim->cval = 0;
    m_tim->ists = 0;
    m_dma->ctrl_bit.chen = 0;
    m_dma->dtcnt = sz + 1;
    m_dma->maddr = (uint32_t)m_tx_buf;
    m_dma->ctrl_bit.chen = 1;
    return 0;
}

void Pwm::read_digital(uint8_t *data, uint32_t sz)
{
    m_dma->ctrl_bit.chen = 0;
    m_dma->dtcnt = sz;
    m_dma->maddr = (uint32_t)data;
    m_dma->ctrl_bit.chen = 1;
}
