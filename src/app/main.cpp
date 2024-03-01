#include "motor.h"
#include "msp.h"
#include "stdio.h"
#include "protocol.h"
#include "sound.h"
#include "board.h"
#include <string.h>

TimerIf *timer = nullptr;
Protocol *proto = nullptr;
MotorIf *motor = nullptr;
Sound *sound = nullptr;

#if !defined(NDEBUG)
UsartIf *debug_usart = nullptr;
Protocol *debug_proto = nullptr;
GpioIf *debug_pin = nullptr;
#endif

uint8_t escInfoBuffer[64];

enum State
{
    IDLE,
    THROTTLE_DETECTED,
    READY_FOR_ARM,
    ARMED,
    READY_TO_GO,
};

static State state = IDLE;
uint32_t dshot_bits;
Protocol::Type proto_type;
uint32_t throttle;

void print_routine()
{
    static uint32_t run_time = 0;
    if (run_time < timer->now_ms())
    {
        run_time = timer->now_ms() + 500;
        // printf("rpm: %d cur:%lu\n", motor->get_rpm(), motor->get_current());
        // printf("%lu \n", dshot_bits);
        // if (proto_type != Protocol::BRUSHED){
        switch (proto_type)
        {
        case Protocol::SERIAL:
            printf("SERIAL\n");
            break;
        case Protocol::DSHOT:
            printf("DSHOT\n");
            break;
        case Protocol::BRUSHED:
            printf("BRUSHED\n");
            break;
        case Protocol::STD_PWM:
            printf("STD_PWM\n");
            break;
        case Protocol::ONESHOT:
            printf("ONESHOT\n");
            break;
        case Protocol::PROSHOT:
            printf("PROSHOT\n");
            break;

        default: // AUTO_DETECT
            printf("NONE\n");
            break;
        }
    }
}

void test(void)
{
    // PwmIf* pwm = PwmIf::new_instance(PA6);
    // pwm->set_mode(PwmIf::OUTPUT);
    // pwm->set_freq(1800);
    // // pwm->set_dutycycle(0.5f);
    // uint32_t pulses[] = {500000, 300000, 10000, 1000};
    // while (1)
    // {
    //     timer->delay_ms(10);
    //     pwm->send_pulses(pulses, 4);
    // }

    PwmIf *pwm = PwmIf::new_instance(PA6);
    pwm->set_mode(PwmIf::INPUT);
    pwm->set_freq(2000);
    uint32_t pulses[32];
    while (1)
    {
        pwm->recv_pulses(pulses, 31);
        timer->delay_ms(100);
        if (pwm->recv_pulses() == 31)
        {
            static char buf[256] = {0};
            int len = 0;
            for (uint32_t i = 0; i < 31; i++)
            {
                len += sprintf(buf + len, "%4lu ", pulses[i]);
            }
            buf[len] = '\n';
            buf[len + 1] = 0;
            printf("%s", buf);
        }
    }
}
bool armed = false;
int main(void)
{
    system_init();
    timer = TimerIf::singleton();
    // test();
#if !defined(NDEBUG)
#if DEBUG_PIN != PIN_NONE
    debug_pin = GpioIf::new_instance(DEBUG_PIN);
    debug_pin->set_mode(GpioIf::OUTPUT);
#endif
    debug_usart = UsartIf::new_instance(PB6, PB7, 256000, 1);
    debug_proto = Protocol::singleton(Protocol::SERIAL, PIN_NONE);
#endif
    motor = MotorIf::singleton(MotorIf::BLDC);
    sound = new Sound(motor);
    sound->power_on_tone();
    while (1) // don't make one loop take more than 10us
    {
        // if (__builtin_expect(!armed, false))
        // {
        //     proto_type = Protocol::auto_detect(PA6);
        //     // if (proto_type != Protocol::AUTO_DETECT)
        //     //     sound->throttle_signal_detected_tone();
        // }
        // else
        {
            motor->poll();
            // proto->poll();
            print_routine();
        }
#if !defined(NDEBUG)
        debug_proto->poll();
#endif
    }
}
