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
    freq = 1800;
    volume = MotorIf::VOLUME_LOW;
    duration = 500;
    state = 0;
}

void Sound::armed_tone(void)
{
    freq = 2200;
    volume = MotorIf::VOLUME_LOW;
    duration = 500;
    state = 0;
}

void Sound::poll(void)
{
    uint32_t now = timer->now_ms();
    if (now < run_time)
        return;

    run_time = now + 1;
    switch (state)
    {
    case 0:
        run_time = now + 500;
        state = 1;
        return;
    case 1:
        motor->beep(freq, volume);
        run_time = now + duration;
        state = 2;
        return;
    case 2:
        motor->beep(freq, MotorIf::VOLUME_OFF);
        state = 3;

    default:
        break;
    }
}
