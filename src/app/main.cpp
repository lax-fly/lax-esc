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
int throttle;

#define PRINT_RPM 1
#define PRINT_DSHOT_DATA 2
#define PRINT_PROTO 3
#define PRINT_DEBUG PRINT_DSHOT_DATA

void print_routine()
{
    static uint32_t run_time = 0;
    if (run_time < timer->now_ms())
    {
        run_time = timer->now_ms() + 500;
#if PRINT_DEBUG == PRINT_RPM
        printf("rpm: %d cur:%lu\n", motor->get_rpm(), motor->get_current());
#elif PRINT_DEBUG == PRINT_DSHOT_DATA
        printf("%lu \n", dshot_bits);
#elif PRINT_DEBUG == PRINT_PROTO
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
            printf("STD_PWM: %lu\n", (uint32_t)(throttle * 2000));
            break;
        case Protocol::ONESHOT:
            printf("ONESHOT: %lu\n", (uint32_t)(throttle * 2000));
            break;
        case Protocol::PROSHOT:
            printf("PROSHOT\n");
            break;

        default: // AUTO_DETECT
            printf("NONE\n");
            break;
        }
#endif
    }
}

#define PWM_TEST 0
uint32_t pulse;
void pwm_test(void)
{
#if PWM_TEST == 1
    MotorPwmIf *pwm = MotorPwmIf::new_instance(MOS_A_HIGH_PIN, MOS_B_HIGH_PIN, MOS_C_HIGH_PIN);
    pwm->set_freq(1000);
    pwm->set_dutycycle(DUTY_CYCLE(0.5f));
    while (1)
    {
    }
#elif PWM_TEST == 2
    SignalPwmIf *pwm = SignalPwmIf::new_instance(PA6);
    pwm->set_mode(SignalPwmIf::PULSE_OUTPUT_CAPTURE);
    uint32_t pulses[] = {400000, 300000, 10000, 1000};
    while (1)
    {
        timer->delay_ms(10);
        pwm->send_pulses(pulses, 4, 500000);
    }
#elif PWM_TEST == 3
    SignalPwmIf *pwm = SignalPwmIf::new_instance(PA6);
    pwm->set_mode(SignalPwmIf::PULSE_OUTPUT_CAPTURE);
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
#elif PWM_TEST == 4
    SignalPwmIf *pwm = SignalPwmIf::new_instance(PA6);
    pwm->set_mode(SignalPwmIf::UP_PULSE_CAPTURE);
    pwm->set_up_pulse_callback([](uint32_t p)
                               { pulse = p; });
    while (1)
    {
        timer->delay_ms(500);
        printf("%lu\n", pulse);
    }
#endif
}

int main(void)
{
    system_init();
    timer = TimerIf::singleton();
    config.load();

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
#if PWM_TEST > 0
    pwm_test();
#endif
    proto_type = Protocol::auto_detect(PA6);
    proto = Protocol::singleton(proto_type, PA6);
    sound->throttle_signal_detected_tone();
    while (1) // don't make one loop take more than 10us, motor->poll must be called once per <10us
    {
        if (proto->signal_lost())
        {
            proto_type = Protocol::auto_detect(PA6);
            proto = Protocol::singleton(proto_type, PA6);
            sound->throttle_signal_detected_tone();
        }
        motor->poll();
        proto->poll();
        motor->poll();
#if !defined(NDEBUG)
        debug_proto->poll();
        print_routine();
#endif
    }
}
