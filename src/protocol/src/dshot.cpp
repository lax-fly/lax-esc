#include "dshot.h"
#include "board.h"
#include "msp.h"

#include <stdio.h>
#include <assert.h>

/*
  copied from betaflight code: src\main\drivers\dshot_command.h
  DshotSettingRequest (KISS24). Spin direction, 3d and save Settings require 10 requests.. and the TLM Byte must always be high if 1-47 are used to send settings

  3D Mode:
  0 = stop
  48   (low) - 1047 (high) -> negative direction
  1048 (low) - 2047 (high) -> positive direction
 */

typedef enum
{
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEACON1,
    DSHOT_CMD_BEACON2,
    DSHOT_CMD_BEACON3,
    DSHOT_CMD_BEACON4,
    DSHOT_CMD_BEACON5,
    DSHOT_CMD_ESC_INFO, // V2 includes settings
    DSHOT_CMD_SPIN_DIRECTION_1,
    DSHOT_CMD_SPIN_DIRECTION_2,
    DSHOT_CMD_3D_MODE_OFF,
    DSHOT_CMD_3D_MODE_ON,
    DSHOT_CMD_SETTINGS_REQUEST, // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS,
    DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE,
    DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_LED0_ON,                       // BLHeli32 only
    DSHOT_CMD_LED1_ON,                       // BLHeli32 only
    DSHOT_CMD_LED2_ON,                       // BLHeli32 only
    DSHOT_CMD_LED3_ON,                       // BLHeli32 only
    DSHOT_CMD_LED0_OFF,                      // BLHeli32 only
    DSHOT_CMD_LED1_OFF,                      // BLHeli32 only
    DSHOT_CMD_LED2_OFF,                      // BLHeli32 only
    DSHOT_CMD_LED3_OFF,                      // BLHeli32 only
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30, // KISS audio Stream mode on/Off
    DSHOT_CMD_SILENT_MODE_ON_OFF = 31,       // KISS silent Mode on/Off
    DSHOT_CMD_MAX = 47
} dshotCommands_e;

enum State
{
    TO_SEND,
    SENDING,
    RECEIVE,
};

static TimerIf *timer;
static PwmIf *pwm;
static uint64_t run_time;
static uint32_t buffer[32];
static int rd_sz;
static State state;
static uint64_t resp_time;
static uint32_t period45; // 4/5 * period, used for bi dir dshot
static uint32_t half_period;
static uint64_t now_us;

static bool edt_mode = false;         // Extended DShot Telemetry (EDT)
static bool serial_telemetry = false; // use serial telemetry?, on the bf configurator, you should enable esc sensor
static dshotCommands_e last_cmd = DSHOT_CMD_MOTOR_STOP;
static uint8_t cmd_repeat = 0;
static uint8_t beacon_mode = 0;
static bool mode_3d = false;
static bool spin_dir_reverse = false;
static uint32_t throttle = 0;

static bool send_temparature = true;
static bool send_current = true;
static bool send_voltage = true;
static bool send_status = false;
static uint32_t send_temparature_time = 0;
static uint32_t send_current_time = 300;
static uint32_t send_voltage_time = 600;

static uint16_t send_value = 0;
static uint32_t timeout = 0;

extern uint32_t dshot_bits;
extern bool armed;

AdcIf *adc_temp;
AdcIf *adc_volt;
AdcIf *adc_curr;

static void send_telemetry(void);
static void restart(void);
static void proccess(void);
static void receive_dealing();
static void send_dealing();
static uint32_t encode2dshot_bits(uint32_t data);

Dshot::Dshot()
{
    timer = TimerIf::singleton();
    pwm = nullptr;
    run_time = 0;
    state = RECEIVE;
    resp_time = 0;
    adc_temp = AdcIf::new_instance(PIN_MAX);
    adc_volt = AdcIf::new_instance(ADC_BAT_PIN);
    adc_curr = AdcIf::new_instance(ADC_CUR_PIN);
}

void Dshot::bind(Pin pin)
{
    pwm = PwmIf::new_instance(pin);
    pwm->set_mode(PwmIf::PULSE_OUTPUT_CAPTURE);
    restart();
}

void Dshot::release(void)
{
    delete pwm;
    pwm = nullptr;
}

Dshot::~Dshot()
{
    delete pwm;
}

void restart(void)
{
    rd_sz = 0;
    if (state == RECEIVE)
        pwm->recv_pulses(buffer, sizeof(buffer) / sizeof(buffer[0]));
}

void dshot_process(uint32_t dshot_bits)
{
    bool telemetry = !!(dshot_bits & 1);

    ::dshot_bits = dshot_bits;
    int value = (int)dshot_bits >> 1;

    if (value < 48)
    {
        if (value == last_cmd)
            cmd_repeat++;
        else
            cmd_repeat = 1;
        last_cmd = (dshotCommands_e)value;
    }

    // if (value < 37 && armed)
    //     return;

    switch (value) // Commands 0-36 are only executed when motors are stopped.
    {
    case DSHOT_CMD_MOTOR_STOP:
        if (!armed)
            armed = true;
        break;
    case DSHOT_CMD_BEACON1:
    case DSHOT_CMD_BEACON2:
    case DSHOT_CMD_BEACON3:
    case DSHOT_CMD_BEACON4:
    case DSHOT_CMD_BEACON5:
        beacon_mode = value;
        break;
    case DSHOT_CMD_ESC_INFO:
        send_value = 0;
        send_status = true;
        break; // V2 includes settings
    case DSHOT_CMD_SPIN_DIRECTION_NORMAL:
    case DSHOT_CMD_SPIN_DIRECTION_1:
        if (cmd_repeat >= 6)
            spin_dir_reverse = false;

        break;
    case DSHOT_CMD_SPIN_DIRECTION_REVERSED:
    case DSHOT_CMD_SPIN_DIRECTION_2:
        if (cmd_repeat >= 6)
            spin_dir_reverse = true;

        break;
    case DSHOT_CMD_3D_MODE_OFF:
        if (cmd_repeat >= 6)
            mode_3d = false;

        break;
    case DSHOT_CMD_3D_MODE_ON:
        if (cmd_repeat >= 6)
            mode_3d = true;
        break;
    case DSHOT_CMD_SETTINGS_REQUEST: // Currently not implemented by bf
        if (cmd_repeat >= 6)
            ;
        break;
    case DSHOT_CMD_SAVE_SETTINGS:
        if (cmd_repeat >= 6)
        {
        }
        break;
    case DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE:
        if (cmd_repeat >= 6 && !edt_mode)
        {
            edt_mode = true;
            send_value = 0;
            send_status = true;
        }
        break;
    case DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE:
        if (cmd_repeat >= 6 && edt_mode)
        {
            edt_mode = false;
            send_value = 0b000011111111;
            send_status = true;
        }
        break;
        // case DSHOT_CMD_LED0_ON:break;                       // BLHeli32 only
        // case DSHOT_CMD_LED1_ON:break;                       // BLHeli32 only
        // case DSHOT_CMD_LED2_ON:break;                       // BLHeli32 only
        // case DSHOT_CMD_LED3_ON:break;                       // BLHeli32 only
        // case DSHOT_CMD_LED0_OFF:break;                      // BLHeli32 only
        // case DSHOT_CMD_LED1_OFF:break;                      // BLHeli32 only
        // case DSHOT_CMD_LED2_OFF:break;                      // BLHeli32 only
        // case DSHOT_CMD_LED3_OFF:break;                      // BLHeli32 only
        // case DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF:break; // KISS audio Stream mode on/Off
        // case DSHOT_CMD_SILENT_MODE_ON_OFF:break;       // KISS silent Mode on/Off

    default:
        if (value > 47)
            throttle = value - 47;
        break;
    }
}

void proccess(void)
{
    if (rd_sz < 31) // it won't rise at the end of the last pulse
        return;

    resp_time = now_us + 25; // 30us, 5us for error
    uint16_t dshot_bits = 0;

    if (__builtin_expect(!armed, false))
    { // auto calc the period when disarmed
        uint32_t period = 0;
        for (uint32_t i = 0; i < 15; i++)
        {
            register uint32_t high = buffer[i * 2];
            register uint32_t low = buffer[(i * 2) + 1];
            period += high + low;
            if (high > low)
                dshot_bits |= 1;

            dshot_bits <<= 1;
        }
        period /= 15;
        half_period = period / 2;
        period45 = period * 4 / 5;
    }
    else
    {
        for (uint32_t i = 0; i < 15; i++)
        {
            register uint32_t high = buffer[i * 2];
            if (high > half_period)
                dshot_bits |= 1;

            dshot_bits <<= 1;
        }
    }

    uint32_t pulse = buffer[30];
    if (pulse > half_period) // deal with the last bit
        dshot_bits |= 1;
    uint32_t crc_sum = dshot_bits;
    crc_sum ^= crc_sum >> 8;
    crc_sum ^= crc_sum >> 4;
    crc_sum &= 0x0f;
    if (crc_sum == 0) // normal dshot
    {
        dshot_process(dshot_bits >> 4);
        timeout = 0;
    }
    else if (crc_sum == 0xf) // dshot2d
    {
        dshot_process(dshot_bits >> 4);
        send_telemetry();
        timeout = 0;
    }
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
    0b01111,
};

uint32_t encode2dshot_bits(uint32_t data)
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
    uint32_t last_bit = 0;
    encode_data <<= (32 - 20);
    for (uint32_t i = 0; i < 20; i++)
    {
        dshot_bits <<= 1;
        if (encode_data & 0x80000000)
        {
            last_bit = !last_bit;
            dshot_bits |= last_bit;
        }
        else
        {
            dshot_bits |= last_bit;
        }
        encode_data <<= 1;
    }
    return dshot_bits;
}

void send_package(void)
{
    state = TO_SEND;

    uint32_t data = 0;
    data = encode2dshot_bits(send_value);

    data <<= 32 - 21;
    uint32_t pulse1 = 0;
    uint32_t pulse0 = period45;
    for (uint32_t i = 0; i < 21; i++)
    { // msb first
        if (data & 0x80000000)
            buffer[i] = pulse1;
        else
            buffer[i] = pulse0;
        data <<= 1;
    }
}

void scheduling_telemetry(void)
{
    if (!edt_mode)
        return;

    uint32_t ms = timer->now_ms();
    if (ms > send_current_time)
    {
        send_current_time = ms + 1000;
        send_current = true;
    }
    if (ms > send_voltage_time)
    {
        send_voltage_time = ms + 1000;
        send_voltage = true;
    }
    if (ms > send_temparature_time)
    {
        send_temparature_time = ms + 2000;
        send_temparature = true;
    }
}

void send_telemetry(void)
{
    if (send_status)
    {
        send_status = false;
        send_value = 0xE00 | send_value;
        send_package();
        return;
    }

    if (!edt_mode)
        goto SEND_ERPM;

    if (send_current)
    {
        send_current = false;
        send_value = 1;                    // adc_curr->sample_voltage();
        send_value = 0x600 | (send_value); // step 1A
    }
    else if (send_voltage)
    {
        send_voltage = false;
        send_value = 12;                       // adc_volt->sample_voltage() / 250;
        send_value = 0x400 | (send_value * 4); // step 0.25v
    }
    else if (send_temparature)
    {
        send_temparature = false;
        send_value = 26;                 // adc_temp->sample_temperature();
        send_value = 0x200 | send_value; // step 1 Celsius degree
    }
    else
    {
        goto SEND_ERPM;
    }
    send_package();
    return;

SEND_ERPM: // actually, the data sended is the period of erpm (1/erpm)
    // format e e e m m m m m m m m m,  erpm = M << E
    uint32_t E = 0;
    send_value = 1000000 / 1960; // 1000000/erpm us
    while (send_value > ((1 << 9) - 1))
    {
        send_value >>= 1;
        E++;
    }
    send_value = (E << 9) + send_value;

    send_package();
}

void receive_dealing()
{
    proccess(); // one byte once to avoid long time cpu occupation
    int rd_sz = pwm->recv_pulses();
    if (rd_sz == 0)
        return;
    if (rd_sz == ::rd_sz)
    { // no byte received over 20us, so restart frame
        restart();
        timeout++;
        return;
    }
    ::rd_sz = rd_sz;
}

void send_dealing()
{
    if (now_us < resp_time)
        return;

    if (state == TO_SEND)
    {
        state = SENDING;
        pwm->send_pulses(buffer, 21, period45);
        return;
    }

    if (pwm->send_pulses() == 0)
    {
        state = RECEIVE;
        restart();
    }
}

void Dshot::poll(void)
{
    if (!pwm)
        return;
    now_us = timer->now_us();
    if (run_time > now_us)
        return;
    run_time += 6; // 6us, the max dshot bit length for 150kHz

    if (state != RECEIVE)
        send_dealing();
    else
    {
        scheduling_telemetry();
        receive_dealing();
    }
}

bool Dshot::signal_lost()
{
    return (timeout > 50000);
}