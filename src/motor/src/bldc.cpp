#include "motor.h"
#include "msp.h"
#include "bldc.h"
#include <assert.h>
#include <stdio.h>

enum Angle
{
    ANGLE_3_75 = 4, // degree 3.75
    ANGLE_7_5 = 3,  // degree 7.5
    ANGLE_15 = 2,   // degree 15
    ANGLE_30 = 1,   // degree 30
};

/*********************** motor parameters *****************************/

static float throttle = 0; // 0~1
static float min_throttle = 0.05f;
static uint32_t polar_cnt = 14;         // the polar count of the target motor
static uint32_t kv = 1400;              // rpm/v while empty load, used to judge current load
static uint32_t blind_interval = 20000; // us
static uint32_t startup_freq = 5000;    // Hz
static Angle commutate_angle = ANGLE_30;
static uint32_t bemf_threshold = 50;    // mV
static uint32_t max_pwm_freq = 50000; // Hz
static uint32_t current_gain = 40;

/**********************************************************************/

#define MIN_VALID_ERPM 8000

#define ARRAY_CNT(x) (sizeof(x) / sizeof(x[0]))

static void set_dutycycle(float dutycycle);
static void set_frequency(uint32_t pwm_freq);
static int Commutate(int step);
static void power_down(void);
static int zero_cross_check(int current_step);
static void update_state(void);
static int get_nxt_step(int current_step, int edge);
static void routine_1kHz(void *data);

static ComparatorIf *cmp_a;
static ComparatorIf *cmp_b;
static ComparatorIf *cmp_c;

static AdcIf *adc_a;
static AdcIf *adc_b;
static AdcIf *adc_c;
static AdcIf *adc_bat;
static AdcIf *adc_cur;

static PwmIf *ha;
static PwmIf *hb;
static PwmIf *hc;

static GpioIf *la;
static GpioIf *lb;
static GpioIf *lc;

static TimerIf *timer;

#ifndef NDEBUG
static GpioIf *debug_pin;
#endif

static float batery_voltage = 12; // default 12v, will be updated when adc sampled the voltage
static uint32_t erpm = 0;
static uint32_t pwm_freq = 0;
static float pwm_dutycycle = 0;
static uint32_t demag = UINT32_MAX;
static uint32_t speed_change_limit = 0;

static uint64_t next_zero = 0;
static uint64_t next_commutate_time;
static uint32_t zero_interval;
static uint64_t now;

static char is_pwm_changed = 1;
static bool is_armed = false;

void BC(void)
{
    ha->disable();
    la->unset();
    hb->enable();
    lb->unset();
    hc->disable();
    lc->set();
}

void AC(void)
{
    ha->enable();
    la->unset();
    hb->disable();
    lb->unset();
    hc->disable();
    lc->set();
}

void AB(void)
{
    ha->enable();
    la->unset();
    hb->disable();
    lb->set();
    hc->disable();
    lc->unset();
}

void CB(void)
{
    ha->disable();
    la->unset();
    hb->disable();
    lb->set();
    hc->enable();
    lc->unset();
}

void CA(void)
{
    ha->disable();
    la->set();
    hb->disable();
    lb->unset();
    hc->enable();
    lc->unset();
}

void BA(void)
{
    ha->disable();
    la->set();
    hb->enable();
    lb->unset();
    hc->disable();
    lc->unset();
}

void set_dutycycle(float dutycycle)
{
    assert(dutycycle <= 1);
    ha->set_dutycycle(dutycycle);
    hb->set_dutycycle(dutycycle);
    hc->set_dutycycle(dutycycle);
}

void set_frequency(uint32_t pwm_freq)
{
    assert(pwm_freq <= 100000);
    ha->set_freq(pwm_freq);
    hb->set_freq(pwm_freq);
    hc->set_freq(pwm_freq);
}

/**
 * the first column indicates which phases are being powered, BC means the phase B is connected to the vbat, and the phase C to the GND
 * the 'next step' column indicates the next step to be powered when falling edge is detected by adc, -1 means falling edge while 1 meas rising.
 *
 * |              | next step at  | next step at | next step at | which adc used | which cmp used | which pwm is |
 * | current step | falling edge  | rising edge  | stalling     | by float pin   | by float pin   | being used   |
 * |--------------|-------------- |--------------|--------------|----------------|----------------|--------------|
 * |    0-BC      |     2-AB      |    1-AC      |     2-AB     |   adc_a        |     cmp_a      |     hb       |
 * |    1-AC      |     2-AB      |    3-CB      |     3-CB     |   adc_b        |     cmp_b      |     ha       |
 * |    2-AB      |     4-CA      |    3-CB      |     4-CA     |   adc_c        |     cmp_c      |     ha       |
 * |    3-CB      |     4-CA      |    5-BA      |     5-BA     |   adc_a        |     cmp_a      |     hc       |
 * |    4-CA      |     0-BC      |    5-BA      |     0-BC     |   adc_b        |     cmp_b      |     hc       |
 * |    5-BA      |     0-BC      |    1-AC      |     1-AC     |   adc_c        |     cmp_c      |     hb       |
 *
 */

static CommutateMap commutate_matrix[6] = {0};

Bldc::Bldc()
{
    timer = TimerIf::singleton();
#ifndef NDEBUG
    debug_pin = GpioIf::new_instance(PA6);
#endif
    ha = PwmIf::new_instance(PA10);
    hb = PwmIf::new_instance(PA9);
    hc = PwmIf::new_instance(PA8);
    la = GpioIf::new_instance(PB1);
    lb = GpioIf::new_instance(PB0);
    lc = GpioIf::new_instance(PA7);
    la->set_mode(GpioIf::OUTPUT);
    lb->set_mode(GpioIf::OUTPUT);
    lc->set_mode(GpioIf::OUTPUT);
    cmp_a = ComparatorIf::new_instance(PA5, PA2, PIN_NONE);
    cmp_b = ComparatorIf::new_instance(PA1, PA2, PIN_NONE);
    cmp_c = ComparatorIf::new_instance(PA0, PA2, PIN_NONE);
    adc_a = AdcIf::new_instance(PA5);
    adc_b = AdcIf::new_instance(PA1);
    adc_c = AdcIf::new_instance(PA0);
    adc_bat = AdcIf::new_instance(PA3);
    adc_cur = AdcIf::new_instance(PA4);
    commutate_matrix[0] = {BC, 2, 1, 2, cmp_a, adc_a, hb};
    commutate_matrix[1] = {AC, 2, 3, 3, cmp_b, adc_b, ha};
    commutate_matrix[2] = {AB, 4, 3, 4, cmp_c, adc_c, ha};
    commutate_matrix[3] = {CB, 4, 5, 5, cmp_a, adc_a, hc};
    commutate_matrix[4] = {CA, 0, 5, 0, cmp_b, adc_b, hc};
    commutate_matrix[5] = {BA, 0, 1, 1, cmp_c, adc_c, hb};

    timer->timing_task_1ms(routine_1kHz, nullptr);
#ifndef NDEBUG
    debug_pin->set_mode(GpioIf::OUTPUT);
#endif
}

void routine_1kHz(void *data)   // this determines pwm update frequency
{
    if (throttle < min_throttle / 2) // throttle below min_throttle / 2 is treated as dead area
        pwm_dutycycle = 0;
    else if (throttle < min_throttle) // motor needs at least min_throttle throttle to startup
        pwm_dutycycle = min_throttle;
    else if (pwm_dutycycle < throttle)
        pwm_dutycycle += 0.02f; // limit speed up rate
    else
        pwm_dutycycle = throttle; // speeding down immediately is permitted

    // prevent motor from burning when stuck(or heavily loaded), the dutycycle will only cause motor to beep when stuck
    if (erpm < (uint32_t)(pwm_dutycycle * batery_voltage * kv) * polar_cnt / 2 / 4)
        pwm_dutycycle = min_throttle;

    pwm_freq = startup_freq + erpm; // e.g. 1400kV motor using 3S -> max erpm = 117600
    if (pwm_freq > max_pwm_freq)    // limit the max pwm freqence
        pwm_freq = max_pwm_freq;

    if (!is_armed)
        pwm_dutycycle = 0;
    is_pwm_changed = 1;
}

void update_state(void)
{
    static uint32_t i = 0;
    static uint32_t buf[8] = {0};
    static uint32_t sum = 0;
    static uint64_t last_us = 0;
    uint32_t intval = now - last_us;
    last_us = now;

    if (intval > blind_interval)
        intval = blind_interval;

    sum = sum - buf[i] + intval;

    if (buf[i] != 0)
        zero_interval = sum / ARRAY_CNT(buf);
    else
        zero_interval = sum / (i + 1);

    buf[i++] = intval;
    if (i >= ARRAY_CNT(buf))
        i = 0;

    next_commutate_time = now + (zero_interval >> commutate_angle);
    next_zero = now + zero_interval;

    if (zero_interval == blind_interval)
        erpm = 0;
    else
        erpm = 60 * 1000000 / 6 / zero_interval;

    speed_change_limit = zero_interval / 2 + zero_interval / 4;
}

volatile uint32_t com_time = 0; // the time when commutating happened
int Commutate(int step)
{
    CommutateMap *com_mtx = &commutate_matrix[step];

    if (demag != UINT32_MAX) // commutate using degree delay when bemf is valid
    {
        if (now < next_commutate_time)
            return -1;
    }
    com_mtx->commutate();
    com_time = now;
    com_mtx->cmp->cmp_result();
    com_mtx->adc->sample_voltage(); // first call to enable channel
    return 0;
}

int zero_cross_check(int current_step)
{
    CommutateMap *com_mtx = &commutate_matrix[current_step];

    uint32_t T = 1000000 / pwm_freq;
    if (now < com_time + T) // the first period after commutatation is invalid for the demagnetic time may make the bemf unusable
        return 0;

    PwmIf *pwm = com_mtx->pwm;
    uint32_t duty = pwm->duty();
    uint32_t cycle = pwm->cycle();
    uint32_t pos = pwm->pos();
    uint32_t error = 2 * cycle / T; // make 1us error, based on the cmp's ouput delay along with the startup dutycycle and frequency
    bool stall = 0;
    do  // software cmp blanking
    {
        if (pos + error > cycle)
        {
            demag = UINT32_MAX;
        }
        else if (pos > demag)
        {
            break;
        }
        else if (pos > duty + error)
        {                                                    // detect demagnatic time automatically
            uint32_t tmp = com_mtx->adc->sample_voltage();     // costs about 1us
            if (tmp > bemf_threshold) // check if the bemf is large enough
                demag = pos;
        }
        else if (pos + error > duty)
        {
            demag = UINT32_MAX;
        }
        else if (pos > error)
        { // this branch is never reachable when frequency is over 25kHz
            demag = UINT32_MAX;
            break;
        }

        if (now > next_zero + speed_change_limit) // limitation for speeddown
        {
            stall = 1;
            break;
        }

        return 0;

    } while (1);

    static int8_t last_res = -1;
    static int8_t cmp_res = -1;

    if (stall)
        cmp_res = !last_res;
    else
    {
        last_res = cmp_res;
        cmp_res = com_mtx->cmp->cmp_result();
        if (last_res == -1)
            last_res = cmp_res;
    }

    if (now < next_zero - speed_change_limit) // limitation for speedup
    {
        cmp_res = last_res;
    }

    int edge = cmp_res - last_res;

#ifndef NDEBUG
    debug_pin->write(cmp_res);
#endif

    if (stall)
        cmp_res = -1;

    return edge;
}

int get_nxt_step(int current_step, int edge)
{
    CommutateMap *com_mtx = &commutate_matrix[current_step];
    if (edge > 0)
    {
        return com_mtx->step_rise;
    }
    else if (edge < 0)
    {
        return com_mtx->step_fall;
    }
    return current_step;
}

void power_down(void)
{
    ha->disable();
    la->unset();
    hb->disable();
    lb->unset();
    hc->disable();
    lc->unset();
}

uint32_t Bldc::get_rpm() const
{
    return erpm * 2 / polar_cnt;
}

uint32_t Bldc::get_current() const
{
    return adc_cur->sample_voltage() * current_gain;
}

void Bldc::set_throttle(float v)
{
    throttle = v;
}

void Bldc::arm(bool state)
{
    is_armed = state;
}

void Bldc::stop()
{
    ha->disable();
    la->set();
    hb->disable();
    lb->set();
    hc->disable();
    lc->set();
    is_armed = false;
    erpm = 0;
}

int Bldc::set_direction(int dir)
{
    return -1; // not support yet
}

void Bldc::poll()
{
    if (!is_armed)
        return;
    now = timer->now_us();

    static int step = 0;
    static int edge = 0;
    if (is_pwm_changed)
    {
        set_dutycycle(pwm_dutycycle); // changing dutycycle or frequency must be synchronous with the commutation for stablization
        set_frequency(pwm_freq);
        is_pwm_changed = 0;
    }
    if (edge)
    {
        if (Commutate(step) == 0)
        {
            edge = 0; // reset the edge
        }
    } // make sure commutation is never blocked
    else
    {
        edge = zero_cross_check(step);
        if (edge)
        {
            update_state();
            step = get_nxt_step(step, edge);
        }
    }
}

void Bldc::beep(uint32_t freq, VolumeLevel volume)
{
    set_frequency(freq);
    switch (volume)
    {
    case MotorIf::VOLUME_OFF:
        set_dutycycle(0);
        break;
    case MotorIf::VOLUME_LOW:
        set_dutycycle(0.01f);
        break;
    case MotorIf::VOLUME_MID:
        set_dutycycle(0.03f);
        break;
    case MotorIf::VOLUME_HIGH:
        set_dutycycle(0.07f);
        break;

    default:
        assert(false);
        break;
    }
    AC();
}