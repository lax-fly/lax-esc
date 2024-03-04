#include "oneshot.h"
#include "stdio.h"

static volatile uint32_t pulse = 0;
static PwmIf *pwm = nullptr;
static uint32_t frame_err = 0;
static uint32_t min_pulse;
static uint32_t max_pulse;
static TimerIf *timer;
static uint32_t run_time = 0;
static bool pre_arm = false;

extern float throttle;
extern bool armed;

Oneshot::Oneshot()
{
    timer = TimerIf::singleton();
}

Oneshot::~Oneshot()
{
}

void Oneshot::poll(void)
{
    uint32_t now = timer->now_ms();
    if (now > run_time)
    {
        run_time = now + 5;
        frame_err++;
    }
}

bool Oneshot::signal_lost()
{
    return frame_err > 100;
}

uint32_t calc_average(uint32_t data)
{
    static uint32_t sum = 0;
    static uint32_t i = 0;
    static uint32_t buf[100] = {0};
    sum = sum + data - buf[i];
    buf[i++] = data;
    if (i >= 100)
        i = 0;
    return sum / 100;
}

void calibrate_shot(uint32_t low, uint32_t high)
{
    max_pulse = high;
    min_pulse = low;
    uint32_t mid_v = (high + low) / 2;

    if (pulse < mid_v)
        return;

    uint32_t last_time = timer->now_ms();
    uint32_t run_time = 0;
    uint32_t second = 0;
    printf("throttle calibration start\n");
    timer->delay_ms(10);
    printf("sampling max throttle\n");
    while (1)
    {
        uint32_t now = timer->now_ms();
        if (run_time != now)
        { // 1ms per loop
            run_time = now;
            max_pulse = calc_average(pulse);
            if (now - last_time < 200)
                ; // beep on
            else
                ; // beep off
            if (now - last_time >= 1000)
            {
                second++;
                last_time = now;
            }
            if (second >= 3)
                break;
        }
    }
    for (uint32_t i = 0; i < 4; i++)
    {
        // four up tone beep
    }
    printf("max throttle saved, wait for throttle low\n");
    timer->delay_ms(10);
    while (pulse > mid_v)
    {
    }
    printf("sampling min throttle\n");
    last_time = timer->now_ms();
    run_time = 0;
    second = 0;
    while (1)
    {
        uint32_t now = timer->now_ms();
        if (run_time != now)
        { // 1ms per loop
            run_time = now;
            min_pulse = calc_average(pulse);
            if (now - last_time < 200)
                ; // beep on
            else if (now - last_time < 300)
                ; // beep off
            else if (now - last_time < 500)
                ; // beep on
            else
                ; // beep off
            if (now - last_time >= 1000)
            {
                second++;
                last_time = now;
            }
            if (second >= 3)
                break;
        }
    }
    printf("min throttle saved, ready to start or reboot\n");
    timer->delay_ms(10);
    for (uint32_t i = 0; i < 4; i++)
    {
        // four down tone beep
    }
    printf("min throttle saved, ready to arm and start or reboot\n");
    timer->delay_ms(10);
    while (pulse < min_pulse * 1.05f)
    {
    }
    printf("throttle signal detected, then put throttle to 0 to arm esc\n");
    timer->delay_ms(10);
    while (1)
    {
        if (pulse > min_pulse * 1.05f)
            last_time = timer->now_ms();
        if (timer->now_ms() - last_time > 1000)
        {
            armed = true;
            printf("esc armed\n");
            break;
        }
    }
    timer->delay_ms(10);
}

void calibration(void)
{
    uint32_t timeout = timer->now_ms() + 1000;
    timer->delay_ms(10);
    while (pulse < 4500)
    {
        if (timer->now_ms() > timeout)
            return;
    }

    if (pulse < 30000)
    { // multishot
        calibrate_shot(5000, 25000);
        return;
    }
    if (pulse < 37000)
    {
        frame_err++;
        return;
    }
    if (pulse < 89000)
    { // onshot42
        calibrate_shot(42000, 84000);
        return;
    }
    if (pulse < 115000)
    {
        frame_err++;
        return;
    }
    if (pulse < 260000)
    { // oneshot125
        calibrate_shot(125000, 250000);
        return;
    }
    if (pulse < 950000)
    {
        frame_err++;
        return;
    }
    if (pulse < 2050000)
    { // std pwm
        calibrate_shot(1000000, 2000000);
        return;
    }
}

void Oneshot::bind(Pin pin)
{
    pwm = PwmIf::new_instance(pin);
    pwm->set_mode(PwmIf::UP_PULSE_CAPTURE); // measuring range: 4ms
    pwm->set_up_pulse_callback(
        [](uint32_t p)
        {
            pulse = p;
            if (p < min_pulse)
            {
                frame_err++;
                throttle = 0;
            }
            else if (p > max_pulse)
            {
                frame_err++;
                throttle = 1;
            }
            else
            {
                pre_arm = true;
                frame_err = 0;
                throttle = (p - min_pulse) * 1.0f / (max_pulse - min_pulse);
            }
            if (!armed)
            {
                armed = pre_arm && throttle < 0.05f;
            }
        });
    calibration();
    printf("max pulse: %lu min pulse: %lu\n", max_pulse, min_pulse);
    timer->delay_ms(10);
}

void Oneshot::release(void)
{
    delete pwm;
    pwm = nullptr;
}