#include "motor.h"
#include "msp.h"
#include "stdio.h"
#include "protocol.h"
#include "sound.h"
#include "board.h"
#include <string.h>

TimerIf *timer = nullptr;
UsartIf *debug_usart = nullptr;
Protocol *proto = nullptr;
MotorIf *motor = nullptr;
Sound *sound = nullptr;

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
    case Protocol::LOCKED:
    {
        Protocol::Package resp_package = {.cmd = Protocol::ERPM, .value = 0};
        proto->send_package(resp_package);
        break;
    }
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
        if (IDLE == state)
        {
            state = THROTTLE_DETECTED;
            motor->set_throttle(0);
            sound->throttle_signal_detected_tone();
        }
        else if (THROTTLE_DETECTED == state)
        {
            state = ARMED;
            sound->armed_tone();
        }
        else if (ARMED == state)
        {
            float throttle = (float)package.value / 2000;
            motor->set_throttle(throttle);
        }
        break;

    default:
        break;
    }
}

void print_routine()
{
    static uint32_t run_time = 0;
    if (run_time < timer->now_ms())
    {
        run_time = timer->now_ms() + 500;
        printf("rpm: %d cur:%d\n", motor->get_rpm(), motor->get_current());
    }
}

int main(void)
{
    system_init();
    timer = TimerIf::singleton();
#if !defined(NDEBUG) && DEBUG_PIN != PIN_NONE
    debug_pin = GpioIf::new_instance(DEBUG_PIN);
    debug_pin->set_mode(GpioIf::OUTPUT);
#endif
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
        print_routine();
    }
}
