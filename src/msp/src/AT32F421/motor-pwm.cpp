#include "motor-pwm.h"
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
};

static PwmMap pwm_maps[] = {
    // each pin must be different from each other.
    {TMR1, PA8, TMR_SELECT_CHANNEL_1, GPIO_MUX_2},
    {TMR1, PA9, TMR_SELECT_CHANNEL_2, GPIO_MUX_2},
    {TMR1, PA10, TMR_SELECT_CHANNEL_3, GPIO_MUX_2},

    {TMR3, PB4, TMR_SELECT_CHANNEL_1, GPIO_MUX_0},
    {TMR3, PB5, TMR_SELECT_CHANNEL_2, GPIO_MUX_0},
    {TMR3, PB0, TMR_SELECT_CHANNEL_3, GPIO_MUX_1},
    {TMR3, PB1, TMR_SELECT_CHANNEL_4, GPIO_MUX_1},
    {TMR3, PA6, TMR_SELECT_CHANNEL_1, GPIO_MUX_1},
    {TMR3, PA7, TMR_SELECT_CHANNEL_2, GPIO_MUX_1},

    {TMR15, PA2, TMR_SELECT_CHANNEL_1, GPIO_MUX_0},
    {TMR15, PA3, TMR_SELECT_CHANNEL_2, GPIO_MUX_0},
};

MotorPwmIf *MotorPwmIf::new_instance(Pin a, Pin b, Pin c)
{
    MotorPwm *new_pwm = new MotorPwm(a, b, c);
    return new_pwm;
}

MotorPwm::MotorPwm(Pin a, Pin b, Pin c)
{
    uint32_t index[3] = {0};
    Pin pins[3] = {a, b, c};
    for (uint32_t i = 0; i < 3; i++)
    {
        uint32_t &j = index[i];
        for (j = 0; j < ARRAY_CNT(pwm_maps); j++)
        {
            if (pins[i] == pwm_maps[j].pin)
                break;
        }
        assert(j < ARRAY_CNT(pwm_maps));
    }

    assert(pwm_maps[index[0]].periph == pwm_maps[index[1]].periph && pwm_maps[index[1]].periph == pwm_maps[index[2]].periph); // all motor pwm pins should share the same timer for running speed

    Gpio::setup_af(a, Gpio::AF_OUTPUT_PP, pwm_maps[index[0]].af);
    Gpio::setup_af(b, Gpio::AF_OUTPUT_PP, pwm_maps[index[1]].af);
    Gpio::setup_af(c, Gpio::AF_OUTPUT_PP, pwm_maps[index[2]].af);

    tmr_channel_select_type ch1, ch2, ch3;
    ch1 = pwm_maps[index[0]].ch;
    ch2 = pwm_maps[index[1]].ch;
    ch3 = pwm_maps[index[2]].ch;

    pina = a;
    pinb = b;
    pinc = c;

    freq = 0;
    tim_config(pwm_maps[index[0]].periph, ch1, ch2, ch3);

    set_freq(1000);
    set_dutycycle(0);
}

MotorPwm::~MotorPwm()
{
    // release tim
    *tim_div = 0;
}

inline void MotorPwm::tim_config(tmr_type *tim, tmr_channel_select_type ch1, tmr_channel_select_type ch2, tmr_channel_select_type ch3)
{
    uint32_t ch_idx1 = ch1 >> 1;
    uint32_t ch_idx2 = ch2 >> 1;
    uint32_t ch_idx3 = ch3 >> 1;
    // map the timer registers
    tim_cdt1 = &(&tim->c1dt)[ch_idx1];
    tim_cdt2 = &(&tim->c1dt)[ch_idx2];
    tim_cdt3 = &(&tim->c1dt)[ch_idx3];
    tim_pr = &tim->pr;
    tim_div = &tim->div;
    tim_cval = &tim->cval;

    tim_cm1 = &tim->cm1;
    tim_cm2 = &tim->cm2;

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

    tmr_output_config_type tmr_output_struct;
    tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_FORCE_LOW;
    tmr_output_struct.oc_output_state = TRUE;
    tmr_output_struct.occ_output_state = FALSE;
    tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.oc_idle_state = FALSE;
    tmr_output_struct.occ_idle_state = FALSE;
    tmr_output_channel_config(tim, ch1, &tmr_output_struct);
    tmr_output_channel_config(tim, ch2, &tmr_output_struct);
    tmr_output_channel_config(tim, ch3, &tmr_output_struct);

    if (ch_idx1 < 2)
    {
        ch1_cm1 = *tim_cm1 | TMR_OUTPUT_CONTROL_PWM_MODE_A << (4 + 8 * ch_idx1);
        ch1_cm2 = *tim_cm2;
    }
    else
    {
        ch1_cm1 = *tim_cm1;
        ch1_cm2 = *tim_cm2 | TMR_OUTPUT_CONTROL_PWM_MODE_A << (4 + 8 * (ch_idx1 - 2));
    }

    if (ch_idx2 < 2)
    {
        ch2_cm1 = *tim_cm1 | TMR_OUTPUT_CONTROL_PWM_MODE_A << (4 + 8 * ch_idx2);
        ch2_cm2 = *tim_cm2;
    }
    else
    {
        ch2_cm1 = *tim_cm1;
        ch2_cm2 = *tim_cm2 | TMR_OUTPUT_CONTROL_PWM_MODE_A << (4 + 8 * (ch_idx2 - 2));
    }

    if (ch_idx3 < 2)
    {
        ch3_cm1 = *tim_cm1 | TMR_OUTPUT_CONTROL_PWM_MODE_A << (4 + 8 * ch_idx3);
        ch3_cm2 = *tim_cm2;
    }
    else
    {
        ch3_cm1 = *tim_cm1;
        ch3_cm2 = *tim_cm2 | TMR_OUTPUT_CONTROL_PWM_MODE_A << (4 + 8 * (ch_idx3 - 2));
    }

    tmr_output_enable(tim, TRUE);
    tmr_counter_enable(tim, TRUE);
    nvic_irq_enable(TMR3_GLOBAL_IRQn, 0, 0);
}

// dutycycle 0.0000 ~ 1.0
void MotorPwm::set_dutycycle(uint32_t dutycycle)
{
    if (__builtin_expect(this->dutycycle == dutycycle, false))
        return;
    this->dutycycle = dutycycle;
    uint32_t cdt = dutycycle * cycle / 2000;
    *tim_cdt1 = cdt;
    *tim_cdt2 = cdt;
    *tim_cdt3 = cdt;
}

void MotorPwm::set_freq(uint32_t freq)
{
    if (__builtin_expect(this->freq == freq, false)) // the current min and max freq are 1000 and 12MHz, value outside that is illegal
        return;
    this->freq = freq;
    uint32_t div = 1;
    cycle = SYS_CLOCK_FREQ / freq;
    while (__builtin_expect(cycle > DEFAULT_PWM_PERIOD - 1, false))
    {                // at most loop 3 times while freq = 1
        div <<= 4;   // *16
        cycle >>= 4; // /16
    }
    *tim_div = div - 1; // attention: the div value will only take effect in the next cycle, so there is some delay according to the cycle length
    *tim_pr = cycle - 1;
    uint32_t cdt = dutycycle * cycle / 2000;
    *tim_cdt1 = cdt;
    *tim_cdt2 = cdt;
    *tim_cdt3 = cdt;
}

uint32_t MotorPwm::get_duty() const
{
    return *tim_cdt;
}

uint32_t MotorPwm::get_cycle() const
{
    return cycle;
}

uint32_t MotorPwm::get_pos() const
{
    return *tim_cval;
}

void MotorPwm::select(Pin pin)
{
    if (pin == pina)
    {
        *tim_cm1 = ch1_cm1;
        *tim_cm2 = ch1_cm2;
        tim_cdt = tim_cdt1;
    }
    else if (pin == pinb)
    {
        *tim_cm1 = ch2_cm1;
        *tim_cm2 = ch2_cm2;
        tim_cdt = tim_cdt2;
    }
    else if (pin == pinc)
    {
        *tim_cm1 = ch3_cm1;
        *tim_cm2 = ch3_cm2;
        tim_cdt = tim_cdt3;
    }else
    {
        *tim_cm1 = 0;
        *tim_cm2 = 0;
    }
}
