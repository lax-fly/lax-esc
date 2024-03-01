#include "dshot.h"
#include "board.h"

#include <stdio.h>
#include <assert.h>

Dshot::Dshot()
{
    timer = TimerIf::singleton();
    // GpioIf *io = GpioIf::new_instance(PA9);
    pwm = PwmIf::new_instance(SIGNAL_IN_PIN);
    pwm->set_mode(PwmIf::INPUT);
    run_time = 0;
    callback = nullptr;
    state = RECEIVE;
    resp_time = 0;
    restart();
}

Dshot::~Dshot()
{
    if (pwm)
        delete pwm;
}

void Dshot::restart(void)
{
    period = 0;
    package.cmd = NONE;
    dshot_bit = rd_sz = 0;
    if (state == RECEIVE)
        pwm->recv_pulses(rx_buf, sizeof(rx_buf) / sizeof(rx_buf[0]));
}

void Dshot::proccess(void)
{
    if (rd_sz < 31) // it won't rise at the end of the last pulse
        return;

    resp_time = now_us + 25; // 30us, 5us for error

    for (uint32_t i = 0; i < 15; i++)
    {
        register uint32_t high = rx_buf[i * 2];
        register uint32_t low = rx_buf[(i * 2) + 1];
        period += high + low;
        if (high > low)
            dshot_bit |= 0x8000 >> i;
    }
    period /= 15;
    uint32_t pulse = rx_buf[30];
    if (pulse > period / 2) // deal with the last bit
        dshot_bit |= 1;
    uint32_t crc_sum = dshot_bit;
    crc_sum ^= crc_sum >> 8;
    crc_sum ^= crc_sum >> 4;
    if ((crc_sum & 0x0f) == 0)
    {
        package.telemetry = !!(dshot_bit & (1 << 4));
        int value = (int)dshot_bit >> 5;
        if (value > 47)
        {
            package.cmd = LOCKED; // THROTTLE;
            package.value = value - 47;
        }
        else if (value == 0)
        {
            package.cmd = LOCKED;
            package.value = 0;
        }
        else if (value < 6)
        {
            package.cmd = BEEP;
            package.value = value;
        }
        else if (value == 6)
        {
            package.cmd = VERSION;
            package.value = 1;
        }
        else if (value == 7)
        {
            package.cmd = DIR;
            package.value = -1;
        }
        else if (value == 8)
        {
            package.cmd = DIR;
            package.value = 1;
        }
        else if (value == 9)
        {
            package.cmd = MODE_3D;
            package.value = 0;
        }
        else if (value == 10)
        {
            package.cmd = MODE_3D;
            package.value = 1;
        }
        else if (value == 11)
        {
            package.cmd = SETTING;
            package.value = value;
        }
        else if (value == 12)
        {
            package.cmd = SAVE_SETTING;
            package.value = value;
        }
        if (callback)
            callback(package);
    }
    restart();
}

void Dshot::set_package_callback(CallBack callback)
{
    this->callback = callback;
}

const char gcr_table[16] = {
    0b11001,
    0b11011,
    0b10010,
    0b10011,
    0b11101,
    0b10101,
    0b10110,
    0b10111,
    0b11010,
    0b01001,
    0b01010,
    0b01011,
    0b11110,
    0b01101,
    0b01110,
    0b01111};

uint32_t Dshot::encode2dshot_bits(uint32_t data)
{
    uint32_t crc_sum = data & 0xf;
    crc_sum ^= (data & 0xf0) >> 4;
    crc_sum ^= (data & 0xf00) >> 8;
    crc_sum = ~crc_sum & 0xf;
    data = (data << 4) | crc_sum;
    uint32_t encode_data = gcr_table[data & 0xf];
    encode_data |= gcr_table[(data & 0xf0) >> 4] << 5;
    encode_data |= gcr_table[(data & 0xf00) >> 8] << 10;
    encode_data |= gcr_table[(data & 0xf000) >> 12] << 15;
    uint32_t dshot_bits = 0;
    encode_data <<= (32 - 20);
    for (uint32_t i = 0; i < 20; i++)
    {
        if (encode_data & 0x80000000)
        {
            dshot_bits = (dshot_bits << 1) + ((dshot_bits & 0x01) ^ 0x1);
        }
        else
        {
            dshot_bits = (dshot_bits << 1) + (dshot_bits & 0x01);
        }
        encode_data <<= 1;
    }
    return dshot_bits;
}

void Dshot::send_package(const Protocol::Package &pakcage)
{
    state = TO_SEND;
    pwm->set_mode(PwmIf::OUTPUT);

    uint32_t data = 0;
    switch (pakcage.cmd)
    {
    case Protocol::STATE_EVENT:
        data = 0xE00;
        break;
    case Protocol::ERPM:
    {
        // format e e e m m m m m m m m m,  erpm = M << E
        data = package.value;
        uint32_t E = 0;
        while (data > ((1 << 9) - 1))
        {
            data >>= 1;
            E++;
        }
        data = (E << 9) + data;
        break;
    }
    case Protocol::TEMPERATURE:
        data = 0x200 | package.value; // step 1℃
        break;
    case Protocol::VOLTAGE:
        data = 0x400 | (package.value / 4); // step 0.25v, value in mV
        break;
    case Protocol::CURRENT:
        data = 0x600 | (package.value / 1000); // step 1A, value in mA
        break;

    default:
        assert(false);
        break;
    }

    data = encode2dshot_bits(data);

    data <<= 32 - 21;
    if (!period)
        period = 1667; // 600kHz
    uint32_t period_54 = period * 4 / 5;
    pwm->set_freq(1000000000 / period_54);
    uint32_t pulse0 = period_54 / 4;
    uint32_t pulse1 = pulse0 + period_54 / 2;
    for (uint32_t i = 0; i < 21; i++)
    { // msb first
        if (data & 0x80000000)
            tx_buf[i] = pulse1;
        else
            tx_buf[i] = pulse0;
        data <<= 1;
    }
}

void Dshot::receive_dealing()
{
    proccess(); // one byte once to avoid long time cpu occupation
    int rd_sz = pwm->recv_pulses();
    if (rd_sz == 0)
        return;
    if (rd_sz == this->rd_sz)
    { // no byte received over 20us, so restart frame
        restart();
        return;
    }
    this->rd_sz = rd_sz;
}

void Dshot::send_dealing()
{
    if (now_us < resp_time)
        return;

    if (state == TO_SEND)
    {
        state = SENDING;
        pwm->send_pulses(tx_buf, 21);
        return;
    }

    if (pwm->send_pulses(nullptr, 0) == 0)
    {
        state = RECEIVE;
        pwm->set_mode(PwmIf::INPUT);
        restart();
    }
}

void Dshot::poll(void)
{
    now_us = timer->now_us();
    if (run_time > now_us)
        return;
    run_time += 5;
    if (state != RECEIVE)
        send_dealing();
    else
        receive_dealing();
}