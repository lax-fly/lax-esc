#include "protocol.h"
#include "serial.h"
#include "dshot.h"

#include <assert.h>
#include "string.h"

static Serial *serial = nullptr;
static Dshot *dshot = nullptr;

static void reset_protocol(void)
{
    if (dshot)
        dshot->release();
}

Protocol::Type Protocol::auto_detect(Pin pin)
{
    static Protocol::Type type = BRUSHED;
    TimerIf *timer = TimerIf::singleton();
    uint32_t pulses[64] = {0};
    reset_protocol();
    PwmIf *pwm = PwmIf::new_instance(pin);
    pwm->set_freq(250); // measuring range: 4ms
    timer->delay_ms(5); // necessary delay
    while (1)
    {
        pwm->recv_pulses(pulses, 32);
        while (pwm->recv_pulses() < 32)
            ;

        if (pulses[0] + pulses[1] < 10000 || pulses[2] + pulses[3] < 10000)
        {
            type = DSHOT;

            uint32_t cnt = 0;
            for (uint32_t i = 0; i < 32; i++)
            {
                if (pulses[i] < 30000)
                    cnt++;
                else
                    cnt = 0;
            }
            if (cnt == 8)
            {
                type = PROSHOT;
            }

            break;
        }

        if (pulses[0] < 300000 || pulses[1] < 300000 || pulses[2] < 300000)
        {
            type = ONESHOT;
            break;
        }

        delete pwm;
        pwm = nullptr;

        // STD_PWM or BRUSHED
        GpioIf *io = GpioIf::new_instance(pin);
        while (io->read() == 0)
        {
        }
        uint64_t start = timer->now_us();
        uint32_t cnt = 0;
        while (io->read() == 1)
        {
            cnt++;
            while (timer->now_us() < start + cnt * 100)
            {
            }
        }
        uint32_t max_throttle;
        uint32_t min_throttle;
        if (cnt > 9 && cnt < 21)
        {
            type = STD_PWM;
        }
        delete io;
        break;
    }

    delete pwm;
    return type;
}

Protocol *Protocol::singleton(Type type, Pin pin)
{
    switch (type)
    {
    case SERIAL:
        if (serial)
            return serial;
        return serial = new Serial();
        break;
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
        break;
    case PROSHOT:
        break;

    default: // AUTO_DETECT
        break;
    }
    // assert(false);
    return nullptr;
}