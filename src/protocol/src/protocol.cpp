#include "protocol.h"
#include "serial.h"
#include "dshot.h"
#include "oneshot.h"

#include <assert.h>
#include "string.h"

#ifndef NDEBUG
static Serial *serial = nullptr;
#endif
static Dshot *dshot = nullptr;
static Oneshot *oneshot = nullptr;

static void reset_protocol(void)
{
    if (dshot)
        dshot->release();
    if (oneshot)
        oneshot->release();
}

volatile uint32_t pulse_cnt = 0;
static uint32_t pulses[16] = {0};
#define ARRAY_SZ(x) (sizeof(x) / sizeof(x[0]))
Protocol::Type Protocol::auto_detect(Pin pin)
{
    TimerIf *timer = TimerIf::singleton();
    reset_protocol();
    SignalPwmIf *pwm = SignalPwmIf::new_instance(pin);
    pwm->set_mode(SignalPwmIf::UP_PULSE_CAPTURE); // measuring range: 4ms
    timer->delay_ms(5);                           // necessary delay
    pwm->set_up_pulse_callback(
        [](uint32_t p)
        {
            if (pulse_cnt < ARRAY_SZ(pulses))
                pulses[pulse_cnt++] = p;
        });
    Protocol::Type type = BRUSHED;
    Protocol::Type last_type = BRUSHED;
    uint32_t type_cnt = 1;
REPEAT:
    while (1)
    {
        pulse_cnt = 0;
        uint32_t start;
        uint32_t period[4] = {0};
        while (pulse_cnt < 1)
            ;
        start = timer->now_us();
        while (pulse_cnt < 2)
            ;
        period[0] = timer->now_us() - start;
        while (pulse_cnt < 3)
            ;
        period[1] = timer->now_us() - start - period[0];
        while (pulse_cnt < 8)
            ;
        period[2] = timer->now_us() - start;
        while (pulse_cnt < 15)
            ;
        period[3] = timer->now_us() - start - period[2];

        if (period[0] < 10 || period[1] < 10)
        {
            uint32_t min = period[0] > period[1] ? period[1] : period[0];
            type = DSHOT;
            if (period[2] > min * 7 + 10 && period[3] > min * 7 + 10)
                type = PROSHOT;
            break;
        }

        if ((pulses[0] < 270000 && pulses[0] > 4500) ||
            (pulses[1] < 270000 && pulses[1] > 4500))
        {
            type = ONESHOT;
            break;
        }

        if ((pulses[0] > 950000 && pulses[0] < 2050000) ||
            (pulses[1] > 950000 && pulses[1] < 2050000))
        {
            type = STD_PWM;
            break;
        }

        if ((pulses[0] < 700000 && pulses[0] > 300000) ||
            (pulses[1] < 700000 && pulses[1] > 300000))
        {
            type = BRUSHED;
            break;
        }
    }

    if (type == last_type)
        type_cnt++;
    else
        type_cnt = 1;
    last_type = type;
    if (type_cnt < 3) // if the detected protocol is the same 3 times continuously, then the protocol is comformed
        goto REPEAT;

    delete pwm;
    return type;
}

Protocol *Protocol::singleton(Type type, Pin pin)
{
    switch (type)
    {
#ifndef NDEBUG
    case SERIAL:
        if (serial)
            return serial;
        return serial = new Serial();
        break;
#endif
    case DSHOT:
        if (!dshot)
            dshot = new Dshot();
        dshot->release();
        dshot->bind(pin);
        return dshot;
    case BRUSHED:
        break;
    case STD_PWM:
    case ONESHOT:
        if (!oneshot)
            oneshot = new Oneshot();
        oneshot->release();
        oneshot->bind(pin);
        return oneshot;
        break;
    case PROSHOT:
        break;

    default: // AUTO_DETECT
        break;
    }
    // assert(false);
    return nullptr;
}