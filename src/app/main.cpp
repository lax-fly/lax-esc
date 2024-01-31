#include "motor.h"
#include "msp.h"
#include "stdio.h"
#include "protocol.h"
#include "sound.h"

TimerIf *timer;
UsartIf *debug_usart;
Protocol *proto;
MotorIf *motor;
Sound *sound;

enum State
{
    IDLE,
    THROTTLE_DETECTED,
    READY_FOR_ARM,
    ARMED,
    READY_TO_GO,
};
static State state = IDLE;

void depackage(const Protocol::Package &package)
{
    switch (package.cmd)
    {
    case Protocol::BEEP:
        break;
    case Protocol::VERSION:
        break;
    case Protocol::DIR:
        break;
    case Protocol::MODE_3D:
        break;
    case Protocol::SETTING:
        break;
    case Protocol::SAVE_SETTING:
        break;
    case Protocol::THROTTLE:
        if (IDLE == state && package.value < 10)
        {
            state = THROTTLE_DETECTED;
            motor->set_throttle(0);
            sound->throttle_signal_detected_tone();
        }
        else if (THROTTLE_DETECTED == state)
        {
            // state = ARMED;
            motor->arm(true);
            // sound->armed_tone();
            static bool flag = false;
            if (flag)
            {
                flag = false;
                motor->set_throttle(0);
            }else
            {
                flag = true;
                motor->set_throttle(0.05f);
            }
        }
        // else if (ARMED == state)
        // {
        //     float thorttle = (float)package.value / 2000;
        //     motor->set_throttle(thorttle);
        // }
        break;

    default:
        break;
    }
}

void print_rpm()
{
    static uint32_t run_time = 0;
    if (run_time < timer->now_ms())
    {
        run_time = timer->now_ms() + 500;
        printf("rpm: %lu cur:%lu\n", motor->get_rpm(), motor->get_current());
    }
}

int main(void)
{
    system_init();
    timer = TimerIf::singleton();
    debug_usart = UsartIf::new_instance(PB6, PB7, 256000, 1);
    proto = Protocol::singleton(Protocol::SERIAL);
    proto->set_package_callback(depackage);
    motor = MotorIf::singleton(MotorIf::BLDC);
    sound = new Sound(motor);
    sound->power_on_tone();
    while (1) // don't make one loop take more than 10us
    {
        motor->poll();
        proto->poll();
        sound->poll();
        print_rpm();
    }
}
