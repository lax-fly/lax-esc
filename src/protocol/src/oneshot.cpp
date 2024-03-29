#include "oneshot.h"
#include "stdio.h"
#include "motor.h"

static volatile int pulse = 0;
static SignalPwmIf *pwm = nullptr;
static uint32_t frame_err = 0;
static int min_pulse;
static int max_pulse;
static TimerIf *timer;
static uint32_t run_time = 0;
static bool data_updated = false;
static int valid_low = 0;
static int valid_high = 0;

extern int throttle;
extern MotorIf *motor;

#define ARRAY_SZ(x) (sizeof(x) / sizeof(x[0]))
#define DEAD_AREA 50

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
        run_time = now + 10;
        frame_err += 10;

        if (data_updated && throttle < DEAD_AREA && throttle > -DEAD_AREA)
            motor->arm(true);
    }
}

bool Oneshot::signal_lost()
{
    bool res = frame_err > 50;
    if (res)
    {
        motor->arm(false);
        pwm->set_up_pulse_callback(nullptr);
        valid_low = valid_high = 0;
    }
    return res;
}

int calc_average(int data)
{
    static uint32_t sum = 0;
    static uint32_t i = 0;
    static uint32_t buf[64] = {0};
    sum = sum + data - buf[i];
    buf[i++] = data;
    if (i >= ARRAY_SZ(buf))
        i = 0;
    return sum / ARRAY_SZ(buf);
}

void calibrate_shot(uint32_t low, uint32_t high)
{
    max_pulse = high;
    min_pulse = low;
    valid_high = high * 1.05f;
    valid_low = low * 0.95f;
    uint32_t tmp_min;
    uint32_t tmp_max;
    int mid_v = (int)(high + low) / 2;
    if (config.mode_3d)
        mid_v = (int)(high + low) * 3 / 4;

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
            tmp_max = calc_average(pulse);
            if (now - last_time < 200)
                motor->beep(TONE5, MotorIf::VOLUME_HIGH); // beep on
            else
                motor->beep(TONE5, MotorIf::VOLUME_OFF);
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
    for (uint32_t i = 0; i < 3; i++)
    {
        motor->beep(TONE2, MotorIf::VOLUME_LOW);
        timer->delay_ms(200);
        motor->beep(TONE3, MotorIf::VOLUME_LOW);
        timer->delay_ms(200);
        motor->beep(TONE4, MotorIf::VOLUME_LOW);
        timer->delay_ms(200);
        motor->beep(TONE5, MotorIf::VOLUME_LOW);
        timer->delay_ms(200);
        motor->beep(TONE5, MotorIf::VOLUME_OFF);
        timer->delay_ms(200);
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
            tmp_min = calc_average(pulse);
            if (now - last_time < 200)
                motor->beep(TONE2, MotorIf::VOLUME_LOW); // beep on
            else if (now - last_time < 300)
                motor->beep(TONE2, MotorIf::VOLUME_OFF); // beep off
            else if (now - last_time < 500)
                motor->beep(TONE2, MotorIf::VOLUME_LOW); // beep on
            else
                motor->beep(TONE2, MotorIf::VOLUME_OFF); // beep off
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
    for (uint32_t i = 0; i < 3; i++)
    {
        motor->beep(TONE5, MotorIf::VOLUME_LOW);
        timer->delay_ms(200);
        motor->beep(TONE3, MotorIf::VOLUME_LOW);
        timer->delay_ms(200);
        motor->beep(TONE4, MotorIf::VOLUME_LOW);
        timer->delay_ms(200);
        motor->beep(TONE2, MotorIf::VOLUME_LOW);
        timer->delay_ms(200);
        motor->beep(TONE2, MotorIf::VOLUME_OFF);
        timer->delay_ms(200);
        // four up tone beep
    }
    printf("min throttle saved, ready to arm and start or reboot\n");
    timer->delay_ms(10);

    max_pulse = tmp_max;
    min_pulse = tmp_min;
    valid_high = max_pulse * 1.01f;
    valid_low = min_pulse * 0.99f;
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
        calibrate_shot(config.multishot_min, config.multishot_max);
        config.multishot_max = max_pulse;
        config.multishot_min = min_pulse;
        return;
    }
    if (pulse < 37000)
    {
        frame_err++;
        return;
    }
    if (pulse < 89000)
    { // onshot42
        calibrate_shot(config.oneshot42_min, config.oneshot42_max);
        config.oneshot42_max = max_pulse;
        config.oneshot42_min = min_pulse;
        return;
    }
    if (pulse < 115000)
    {
        frame_err++;
        return;
    }
    if (pulse < 260000)
    { // oneshot125
        calibrate_shot(config.oneshot125_min, config.oneshot125_max);
        config.oneshot125_max = max_pulse;
        config.oneshot125_min = min_pulse;
        return;
    }
    if (pulse < 950000)
    {
        frame_err++;
        return;
    }
    if (pulse < 2050000)
    { // std pwm
        calibrate_shot(config.pwm_min, config.pwm_max);
        config.pwm_max = max_pulse;
        config.pwm_min = min_pulse;
        return;
    }
}

void Oneshot::bind(Pin pin)
{
    pulse = 0;
    pwm = SignalPwmIf::new_instance(pin);
    pwm->set_mode(SignalPwmIf::UP_PULSE_CAPTURE); // measuring range: 4ms
    pwm->set_up_pulse_callback(
        [](uint32_t p)
        {
            pulse = p;
            if (pulse < valid_low)
            {
                frame_err++;
                return;
            }

            if (pulse > valid_high)
            {
                frame_err++;
                return;
            }

                data_updated = true;
                frame_err = 0;
            if (pulse < min_pulse)
                pulse = min_pulse;
            if (pulse > max_pulse)
                pulse = max_pulse;

            throttle = (pulse - min_pulse) * 2000 / (max_pulse - min_pulse);

            if (config.mode_3d)
            {
                throttle = (throttle - 1000) * 2;
            }

            if (throttle < DEAD_AREA && throttle > -DEAD_AREA) // dead area
                throttle = 0;

            if (config.spin_dir_reverse)
                throttle = -throttle;
            motor->set_throttle(throttle);
        });
    calibration();
    config.save();
    printf("max pulse: %d min pulse: %d\n", max_pulse, min_pulse);
    timer->delay_ms(10);
}

void Oneshot::release(void)
{
    delete pwm;
    pwm = nullptr;
}