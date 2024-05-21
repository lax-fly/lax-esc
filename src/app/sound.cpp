#include "sound.h"

void Sound::power_on_tone(void)
{
    motor->beep(TONE2, MotorIf::VOLUME_LOW);
    timer->delay_ms(200);

    motor->beep(TONE3, MotorIf::VOLUME_LOW);
    timer->delay_ms(200);

    motor->beep(TONE4, MotorIf::VOLUME_LOW);
    timer->delay_ms(200);
    motor->beep(TONE4, MotorIf::VOLUME_OFF);
}

void Sound::throttle_signal_detected_tone(void)
{
    timer->delay_ms(200);
    motor->beep(TONE2, MotorIf::VOLUME_LOW);
    timer->delay_ms(500);
    motor->beep(TONE2, MotorIf::VOLUME_OFF);
}

void Sound::armed_tone(void)
{
    motor->beep(TONE5, MotorIf::VOLUME_LOW);
    timer->delay_ms(500);
    motor->beep(TONE5, MotorIf::VOLUME_OFF);
}
