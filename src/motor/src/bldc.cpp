#include "motor.h"
#include "msp.h"
#include "bldc.h"
#include "board.h"
#include <assert.h>
#include <stdio.h>
#include <utility>

typedef struct
{
    void (*commutate)(void);
    int step_fall;
    int step_rise;
    ComparatorIf *cmp;
    AdcIf *adc;
    PwmIf *pwm;
} CommutateMap;

enum Angle
{
    ANGLE_1_875 = 5, // degree 1.875
    ANGLE_3_75 = 4,  // degree 3.75
    ANGLE_7_5 = 3,   // degree 7.5
    ANGLE_15 = 2,    // degree 15
    ANGLE_30 = 1,    // degree 30
};

/*********************** motor parameters *****************************/

static uint32_t throttle = 0;                     // 0~1
static uint32_t min_throttle = DUTY_CYCLE(0.05f); // for 3S. Grow it if motor won't startup with batery below 3S or bigger startup moment
static uint32_t polar_cnt = 14;                   // the polar count of the target motor
static uint32_t kv = 1400;                        // rpm/v while empty load, used to judge current load
static uint32_t blind_interval = 20000;           // us
static uint32_t startup_freq = 5000;              // Hz
static Angle commutate_angle = ANGLE_30;
static uint32_t bemf_threshold = 50;  // mV
static uint32_t max_pwm_freq = 50000; // Hz
static uint32_t current_gain = 5;     // the real current(mA) value divided by voltage(mV) from adc
static uint32_t voltage_gain = 11;    // the read voltage value divided by voltage(mV) from adc

/**********************************************************************/

#define ARRAY_CNT(x) (sizeof(x) / sizeof(x[0]))

static void set_dutycycle(uint32_t dutycycle);
static void set_frequency(uint32_t pwm_freq);
static int Commutate(int step);
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

#if !defined(NDEBUG) && DEBUG_PIN != PIN_NONE
extern GpioIf *debug_pin;
#endif

static uint32_t batery_voltage = 12000; // default 12v, will be updated when adc sampled the voltage
static uint32_t heavy_load_erpm = 0;    // erpm
static uint32_t turn_dir_erpm = 0;
static uint32_t erpm = 0;
static uint32_t pwm_freq = 0;
static uint32_t pwm_dutycycle = 0;
static uint32_t demag = UINT32_MAX;
static uint32_t speed_change_limit = 0;
static int spin_direction = 0; // 1: forward -1: backward
static bool armed = false;

static uint64_t next_zero = 0;
static uint64_t commutate_time;
static uint32_t zero_interval;
static uint64_t now;

static char is_pwm_changed = 1;
static bool braking = false;

#define VOLTAGE_1S (4200) // mV

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

void set_dutycycle(uint32_t dutycycle)
{
    assert(dutycycle <= 2000);
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
 * |              | next step at  | next step at | which adc used | which cmp used | which pwm is |
 * | current step | falling edge  | rising edge  | by float pin   | by float pin   | being used   |
 * |--------------|-------------- |--------------|----------------|----------------|--------------|
 * |    0-BC      |     2-AB      |    1-AC      |   adc_a        |     cmp_a      |     hb       |
 * |    1-AC      |     2-AB      |    3-CB      |   adc_b        |     cmp_b      |     ha       |
 * |    2-AB      |     4-CA      |    3-CB      |   adc_c        |     cmp_c      |     ha       |
 * |    3-CB      |     4-CA      |    5-BA      |   adc_a        |     cmp_a      |     hc       |
 * |    4-CA      |     0-BC      |    5-BA      |   adc_b        |     cmp_b      |     hc       |
 * |    5-BA      |     0-BC      |    1-AC      |   adc_c        |     cmp_c      |     hb       |
 *
 */
static CommutateMap commutate_matrix[6] = {0};

Bldc::Bldc()
{
    timer = TimerIf::singleton();
    ha = PwmIf::new_instance(MOS_A_HIGH_PIN);
    hb = PwmIf::new_instance(MOS_B_HIGH_PIN);
    hc = PwmIf::new_instance(MOS_C_HIGH_PIN);
    la = GpioIf::new_instance(MOS_A_LOW_PIN);
    lb = GpioIf::new_instance(MOS_B_LOW_PIN);
    lc = GpioIf::new_instance(MOS_C_LOW_PIN);
    la->set_mode(GpioIf::OUTPUT);
    lb->set_mode(GpioIf::OUTPUT);
    lc->set_mode(GpioIf::OUTPUT);
    cmp_a = ComparatorIf::new_instance(CMP_A_POS_PIN, CMP_A_NEG_PIN, CMP_OUT_PIN);
    cmp_b = ComparatorIf::new_instance(CMP_B_POS_PIN, CMP_B_NEG_PIN, CMP_OUT_PIN);
    cmp_c = ComparatorIf::new_instance(CMP_C_POS_PIN, CMP_C_NEG_PIN, CMP_OUT_PIN);
    adc_a = AdcIf::new_instance(ADC_A_PIN);
    adc_b = AdcIf::new_instance(ADC_B_PIN);
    adc_c = AdcIf::new_instance(ADC_C_PIN);
    adc_bat = AdcIf::new_instance(ADC_BAT_PIN);
    adc_cur = AdcIf::new_instance(ADC_CUR_PIN);
    set_throttle(0);
    batery_voltage = adc_bat->sample_voltage() * voltage_gain;
    batery_voltage = 8000; // batery_voltage < VOLTAGE_1S ? VOLTAGE_1S : batery_voltage;
    heavy_load_erpm = batery_voltage * kv / 1000 * polar_cnt / 2 / 4;
    turn_dir_erpm = 450 * polar_cnt / 2;

    min_throttle = 0.8f * 1000 * kv / batery_voltage; // 0.0004 = 0.05 * 11v / 1400kv
    if (min_throttle < DUTY_CYCLE(0.05f))
        min_throttle = DUTY_CYCLE(0.05f);

    timer->timing_task_1ms(routine_1kHz, nullptr);
}

Bldc::~Bldc()
{
}

void routine_1kHz(void *data) // this determines pwm update frequency
{
    if (throttle < min_throttle / 2) // throttle below min_throttle / 2 is treated as dead area
        pwm_dutycycle = 0;
    else if (throttle < min_throttle) // motor needs at least min_throttle throttle to startup
        pwm_dutycycle = min_throttle;
    else if (pwm_dutycycle < throttle)
    { /*
      limit speed up rate, throttle 0->1 requires 100ms.
      careful to grow this, the faster, the easier to stall.
      I've tried 0.02, which is ok, but I use 0.01 for stability.
      */
        pwm_dutycycle += 20;
    }
    else
        pwm_dutycycle = throttle; // speeding down immediately is permitted

    if (pwm_dutycycle > 2000)
        pwm_dutycycle = 2000;

    // prevent motor from burning when stuck(or heavily loaded), the dutycycle will only cause motor to beep when stuck
    if (erpm < (uint32_t)(pwm_dutycycle * heavy_load_erpm / 2000))
        pwm_dutycycle = min_throttle;

    if (braking)
    {
        pwm_dutycycle = 0;
        if (erpm < turn_dir_erpm) // braking util motor is slow enough
            braking = false;
    }

    pwm_freq = startup_freq + erpm; // e.g. 1400kV motor using 3S -> max erpm = 117600
    if (pwm_freq > max_pwm_freq)    // limit the max pwm freqence
        pwm_freq = max_pwm_freq;

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

    commutate_time = now + (zero_interval >> commutate_angle);
    next_zero = now + zero_interval;

    if (intval == blind_interval)
        erpm = 0;
    else
        erpm = 60 * 1000000 / 6 / zero_interval;

    speed_change_limit = zero_interval / 2 + zero_interval / 4;
}

int Commutate(int step)
{
    CommutateMap *com_mtx = &commutate_matrix[step];

    if (demag != UINT32_MAX) // commutate using degree delay when bemf is valid
    {
        if (now < commutate_time)
            return -1;
    }
    com_mtx->commutate();
    com_mtx->cmp->prepare();
    com_mtx->adc->prepare(); // first call to enable channel
    return 0;
}

int zero_cross_check(int current_step)
{
    CommutateMap *com_mtx = &commutate_matrix[current_step];

    uint32_t T = 1000000 / pwm_freq;

    PwmIf *pwm = com_mtx->pwm;
    uint32_t duty = pwm->get_duty();
    uint32_t cycle = pwm->get_cycle();
    uint32_t pos = pwm->get_pos();
    uint32_t error = 2 * cycle / T; // make 1us error, based on the cmp's ouput delay along with the startup dutycycle and frequency
    bool stall = 0;
    do // software cmp blanking
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
        {                                                  // detect demagnatic time automatically
            uint32_t tmp = com_mtx->adc->sample_voltage(); // costs about 1us
            if (tmp > bemf_threshold)                      // check if the bemf is large enough
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

#if !defined(NDEBUG) && DEBUG_PIN != PIN_NONE
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

int Bldc::get_rpm() const
{
    if (spin_direction >= 0)
        return erpm * 2 / polar_cnt;
    else
        return -(erpm * 2 / polar_cnt);
}

int Bldc::get_erpm() const
{
    if (spin_direction >= 0)
        return erpm;
    else
        return -erpm;
}

int Bldc::get_e_period() const
{
    if (erpm == 0)
        return 0xffff;
    return 60 * 1000000 / erpm;
}

int Bldc::get_current() const
{
    return adc_cur->sample_voltage() * current_gain;
}

void Bldc::set_throttle(int v)
{
    bool dir_changed = false;
    if (v >= 0)
    {
        if (spin_direction <= 0)
            dir_changed = true;
        spin_direction = 1;
        throttle = v;
    }
    else
    {
        if (spin_direction > 0)
            dir_changed = true;
        spin_direction = -1;
        throttle = -v;
    }

    if (!dir_changed)
        return;

    stop();
    // swap any two phase to change the spin direction
    if (spin_direction > 0)
    {
        commutate_matrix[0] = {BC, 2, 1, cmp_a, adc_a, hb};
        commutate_matrix[1] = {AC, 2, 3, cmp_b, adc_b, ha};
        commutate_matrix[2] = {AB, 4, 3, cmp_c, adc_c, ha};
        commutate_matrix[3] = {CB, 4, 5, cmp_a, adc_a, hc};
        commutate_matrix[4] = {CA, 0, 5, cmp_b, adc_b, hc};
        commutate_matrix[5] = {BA, 0, 1, cmp_c, adc_c, hb};
    }
    else if (spin_direction < 0)
    {
        commutate_matrix[0] = {CB, 2, 1, cmp_a, adc_a, hc};
        commutate_matrix[1] = {AB, 2, 3, cmp_c, adc_c, ha};
        commutate_matrix[2] = {AC, 4, 3, cmp_b, adc_b, ha};
        commutate_matrix[3] = {BC, 4, 5, cmp_a, adc_a, hb};
        commutate_matrix[4] = {BA, 0, 5, cmp_c, adc_c, hb};
        commutate_matrix[5] = {CA, 0, 1, cmp_b, adc_b, hc};
    }
}

void Bldc::stop()
{
    pwm_dutycycle = 0;
    pwm_freq = startup_freq;
    ha->disable();
    la->set();
    hb->disable();
    lb->set();
    hc->disable();
    lc->set();
    braking = true;
}

void Bldc::poll()
{
    if (!armed)
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
    if (!braking && edge)
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
        set_dutycycle(DUTY_CYCLE(0.01f));
        break;
    case MotorIf::VOLUME_MID:
        set_dutycycle(DUTY_CYCLE(0.03f));
        break;
    case MotorIf::VOLUME_HIGH:
        set_dutycycle(DUTY_CYCLE(0.07f));
        break;

    default:
        assert(false);
        break;
    }
    AC();
}

void Bldc::arm(bool state)
{
    if (state == armed)
        return;

    throttle = 0;
    pwm_dutycycle = 0;
    set_dutycycle(0);

    if (!armed && state)
    {
        beep(TONE5, MotorIf::VOLUME_LOW);
        timer->delay_ms(500);
        beep(TONE5, MotorIf::VOLUME_OFF);
    }
    armed = state;
}

bool Bldc::is_armed()
{
    return armed;
}