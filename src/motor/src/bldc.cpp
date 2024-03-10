#include "motor.h"
#include "msp.h"
#include "bldc.h"
#include "board.h"
#include "config.h"

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
} CommutateMap;

#define ARRAY_CNT(x) (sizeof(x) / sizeof(x[0]))

static void set_dutycycle(uint32_t dutycycle);
static void set_frequency(uint32_t pwm_freq);
static int Commutate(int step);
static int zero_cross_check(int current_step);
static void update_state(void);
static int get_nxt_step(int current_step, int edge);
static void routine_10kHz(void *data);

static ComparatorIf *cmp_a;
static ComparatorIf *cmp_b;
static ComparatorIf *cmp_c;

static AdcIf *adc_a;
static AdcIf *adc_b;
static AdcIf *adc_c;
static AdcIf *adc_bat;
static AdcIf *adc_cur;

static MotorPwmIf *pwms;
static MotorIoIf *io;

static TimerIf *timer;

#if !defined(NDEBUG) && DEBUG_PIN != PIN_NONE
extern GpioIf *debug_pin;
#endif

static uint32_t throttle = 0;           // 0~2000 map to 0.0-1.0
static uint32_t min_throttle;           // for 3S. Grow it if motor won't startup with batery below 3S or bigger startup moment
static uint32_t batery_voltage = 12000; // default 12v, will be updated when adc sampled the voltage
static uint32_t heavy_load_erpm = 0;    // erpm
static uint32_t turn_dir_erpm = 0;
static uint32_t erpm = 0;
static uint32_t pwm_freq = 0;
static uint32_t pwm_period = 0;
static uint32_t pwm_dutycycle = 0;
static uint32_t demag = UINT32_MAX;
static uint32_t speed_change_limit = 0;
static int spin_direction = 1; // 1: forward -1: backward
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
    pwms->select(MOS_B_HIGH_PIN);
    io->select(MOS_C_LOW_PIN);
}

void AC(void)
{
    pwms->select(MOS_A_HIGH_PIN);
    io->select(MOS_C_LOW_PIN);
}

void AB(void)
{
    pwms->select(MOS_A_HIGH_PIN);
    io->select(MOS_B_LOW_PIN);
}

void CB(void)
{
    pwms->select(MOS_C_HIGH_PIN);
    io->select(MOS_B_LOW_PIN);
}

void CA(void)
{
    pwms->select(MOS_C_HIGH_PIN);
    io->select(MOS_A_LOW_PIN);
}

void BA(void)
{
    pwms->select(MOS_B_HIGH_PIN);
    io->select(MOS_A_LOW_PIN);
}

void set_dutycycle(uint32_t dutycycle)
{
    assert(dutycycle <= 2000);
    pwms->set_dutycycle(dutycycle);
}

void set_frequency(uint32_t pwm_freq)
{
    assert(pwm_freq <= 100000);
    pwms->set_freq(pwm_freq);
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

static CommutateMap forward_commutate_matrix[6] = {0};

static CommutateMap backward_commutate_matrix[6] = {0};

void init_commutate_matrix()
{
    forward_commutate_matrix[0] = {BC, 2, 1, cmp_a, adc_a};
    forward_commutate_matrix[1] = {AC, 2, 3, cmp_b, adc_b};
    forward_commutate_matrix[2] = {AB, 4, 3, cmp_c, adc_c};
    forward_commutate_matrix[3] = {CB, 4, 5, cmp_a, adc_a};
    forward_commutate_matrix[4] = {CA, 0, 5, cmp_b, adc_b};
    forward_commutate_matrix[5] = {BA, 0, 1, cmp_c, adc_c};

    // swap any two phase to change the spin direction
    backward_commutate_matrix[0] = {CB, 2, 1, cmp_a, adc_a};
    backward_commutate_matrix[1] = {AB, 2, 3, cmp_c, adc_c};
    backward_commutate_matrix[2] = {AC, 4, 3, cmp_b, adc_b};
    backward_commutate_matrix[3] = {BC, 4, 5, cmp_a, adc_a};
    backward_commutate_matrix[4] = {BA, 0, 5, cmp_c, adc_c};
    backward_commutate_matrix[5] = {CA, 0, 1, cmp_b, adc_b};
}

static CommutateMap *commutate_matrix = forward_commutate_matrix;

Bldc::Bldc()
{
    timer = TimerIf::singleton();
    pwms = MotorPwmIf::new_instance(MOS_A_HIGH_PIN, MOS_B_HIGH_PIN, MOS_C_HIGH_PIN);
    io = MotorIoIf::new_instance(MOS_A_LOW_PIN, MOS_B_LOW_PIN, MOS_C_LOW_PIN);
    cmp_a = ComparatorIf::new_instance(CMP_A_POS_PIN, CMP_A_NEG_PIN, CMP_OUT_PIN);
    cmp_b = ComparatorIf::new_instance(CMP_B_POS_PIN, CMP_B_NEG_PIN, CMP_OUT_PIN);
    cmp_c = ComparatorIf::new_instance(CMP_C_POS_PIN, CMP_C_NEG_PIN, CMP_OUT_PIN);
    adc_a = AdcIf::new_instance(ADC_A_PIN);
    adc_b = AdcIf::new_instance(ADC_B_PIN);
    adc_c = AdcIf::new_instance(ADC_C_PIN);
    adc_bat = AdcIf::new_instance(ADC_BAT_PIN);
    adc_cur = AdcIf::new_instance(ADC_CUR_PIN);
    init_commutate_matrix();
    set_throttle(0);
    batery_voltage = adc_bat->sample_voltage() * config.voltage_gain;
    batery_voltage = batery_voltage < VOLTAGE_1S ? VOLTAGE_1S : batery_voltage;
    heavy_load_erpm = batery_voltage * config.kv / 1000 * config.polar_cnt / 2 / 4;
    turn_dir_erpm = 450 * config.polar_cnt / 2;

    min_throttle = 0.8f * 1000 * config.kv / batery_voltage; // 0.0004 = 0.05 * 11v / 1400kv
    if (min_throttle < DUTY_CYCLE(0.05f))
        min_throttle = DUTY_CYCLE(0.05f);

    timer->timing_task_10kHz(routine_10kHz, nullptr);
}

Bldc::~Bldc()
{
}

void routine_10kHz(void *data) // this determines pwm update frequency
{   // be careful, this function is running in interrupt context
    if (pwm_dutycycle < throttle)
    { /*
      limit speed up rate, throttle 0->2000 requires 100ms.
      careful to grow this, the faster, the easier to stall.
      I've tried 40, which is ok, but I use 20 for stability.
      */
        pwm_dutycycle += 5;
    }
    else
        pwm_dutycycle = throttle; // speeding down immediately is permitted

    if (pwm_dutycycle > 2000)
        pwm_dutycycle = 2000;

    // prevent motor from burning when stuck(or heavily loaded), the dutycycle will only cause motor to beep when stuck
    if (erpm * 2000 < (uint32_t)(pwm_dutycycle * heavy_load_erpm))
        pwm_dutycycle = min_throttle;

    if (braking)
    {
        pwm_dutycycle = 0;
        if (erpm < turn_dir_erpm) // braking util motor is slow enough
            braking = false;
    }

    pwm_freq = config.startup_freq + erpm; // e.g. 1400kV motor using 3S -> max erpm = 117600
    if (pwm_freq > config.max_pwm_freq)    // limit the max pwm freqence
        pwm_freq = config.max_pwm_freq;

    pwm_period = 1000000 / pwm_freq;

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

    if (intval > config.blind_interval)
        intval = config.blind_interval;

    sum = sum - buf[i] + intval;

    zero_interval = sum / ARRAY_CNT(buf);

    buf[i++] = intval;
    if (i >= ARRAY_CNT(buf))
        i = 0;

    if (demag == UINT32_MAX) // commutate using degree delay when bemf is valid
        commutate_time = now + (zero_interval >> ANGLE_1_875);
    else
        commutate_time = now + (zero_interval >> config.commutate_angle);
    next_zero = now + zero_interval;

    if (intval == config.blind_interval)
        erpm = 0;
    else
        erpm = 60 * 1000000 / 6 / zero_interval;

    speed_change_limit = zero_interval / 2 + zero_interval / 4;
}
uint64_t com_time = 0;
int Commutate(int step)
{
    if (now < commutate_time)
        return -1;
    CommutateMap *com_mtx = &commutate_matrix[step];
    if (!braking)
        com_mtx->commutate();
    com_time = now;
    com_mtx->cmp->prepare();
    com_mtx->adc->prepare(); // first call to enable channel
    return 0;
}

int zero_cross_check(int current_step)
{
    CommutateMap *com_mtx = &commutate_matrix[current_step];

    uint32_t duty = pwms->get_duty();
    uint32_t cycle = pwms->get_cycle();
    uint32_t pos = pwms->get_pos();
    uint32_t error = 1 * cycle / pwm_period; // make 1us error, based on the cmp's ouput delay along with the startup dutycycle and frequency
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
        { // detect demagnatic time automatically
            if (now < com_time + zero_interval / 32) // when start up, there is some interference after commutating, which may cause the demag measuring not accurate
                return 0;
            uint32_t tmp = com_mtx->adc->sample_voltage(); // costs about 1us
            if (tmp > config.bemf_threshold)               // check if the bemf is large enough
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

    if (now + speed_change_limit < next_zero) // limitation for speedup
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
    if (spin_direction > 0)
        return erpm * 2 / config.polar_cnt;
    else
        return -(erpm * 2 / config.polar_cnt);
}

int Bldc::get_erpm() const
{
    if (spin_direction > 0)
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
    return adc_cur->sample_voltage() * config.current_gain;
}

void Bldc::set_throttle(int v)
{
    bool dir_changed = false;
    if (v == 0)
    {
        throttle = v;
    }
    else if (v > 0)
    {
        if (spin_direction < 0)
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
    if (spin_direction > 0)
    {
        commutate_matrix = forward_commutate_matrix;
    }
    else if (spin_direction < 0)
    {
        commutate_matrix = backward_commutate_matrix;
    }
}

void Bldc::stop()
{
    pwm_dutycycle = 0;
    pwm_freq = config.startup_freq;
    pwms->select(PIN_NONE);
    io->select(PIN_MAX);
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