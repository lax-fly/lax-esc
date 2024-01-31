#include "sound.h"

void Sound::power_on_tone(void)
{
    motor->beep(1800, MotorIf::VOLUME_LOW);
    timer->delay_ms(200);

    motor->beep(2000, MotorIf::VOLUME_LOW);
    timer->delay_ms(200);

    motor->beep(2200, MotorIf::VOLUME_LOW);
    timer->delay_ms(200);
    motor->beep(2200, MotorIf::VOLUME_OFF);
}

void Sound::throttle_signal_detected_tone(void)
{
    motor->beep(1800, MotorIf::VOLUME_LOW);
    timer->delay_ms(500);
    motor->beep(1800, MotorIf::VOLUME_OFF);
}

void Sound::armed_tone(void)
{
    motor->beep(2200, MotorIf::VOLUME_LOW);
    timer->delay_ms(500);
    motor->beep(2200, MotorIf::VOLUME_OFF);
}
