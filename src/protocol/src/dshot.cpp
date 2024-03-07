#include "dshot.h"
#include "board.h"
#include "msp.h"
#include "motor.h"

#include <stdio.h>
#include <assert.h>

extern MotorIf *motor;
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
    PREPARING,
    PREPARED,
    SENDING,
    RECEIVE,
};

static TimerIf *timer;
static SignalPwmIf *pwm;
static uint64_t run_time;
static uint16_t buffer[32];
static int rd_sz;
static State state;
static uint64_t resp_time;
static uint32_t period45 = 0; // 4/5 * period, used for bi dir dshot
static uint32_t half_period;
static uint64_t now_us;

static dshotCommands_e last_cmd = DSHOT_CMD_MOTOR_STOP;
static uint8_t cmd_repeat = 0;
static int throttle = 0;

static bool send_temparature = false;
static bool send_current = false;
static bool send_voltage = false;
static bool send_status = false;
static uint32_t send_temparature_time = 0;
static uint32_t send_current_time = 300;
static uint32_t send_voltage_time = 600;

static uint16_t send_value = 0;
static uint32_t frame_err = 0;
static uint32_t freq_lock = 0;

extern uint32_t dshot_bits;

AdcIf *adc_temp;
AdcIf *adc_volt;
AdcIf *adc_curr;

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
    pwm = SignalPwmIf::new_instance(pin);
    pwm->set_mode(SignalPwmIf::PULSE_OUTPUT_CAPTURE);
    freq_lock = 0;
    frame_err = 0;
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
    rd_sz = sizeof(buffer) / sizeof(buffer[0]);
    if (state == RECEIVE)
        pwm->recv_pulses(buffer, rd_sz);
}

void dshot_process(uint32_t dshot_bits)
{
    bool telemetry = dshot_bits & 1;

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

    switch (value) // Commands 0-36 are only executed when motors are stopped.
    {
    case DSHOT_CMD_MOTOR_STOP:
        motor->set_throttle(0);
        motor->arm(true);
        break;
    case DSHOT_CMD_BEACON1:
        motor->beep(TONE1, MotorIf::VOLUME_LOW);
        timer->delay_ms(200); // FC will wait for 260ms before next frame according to the edt protocol
        motor->beep(TONE5, MotorIf::VOLUME_OFF);
        break;
    case DSHOT_CMD_BEACON2:
        motor->beep(TONE2, MotorIf::VOLUME_LOW);
        timer->delay_ms(200); // FC will wait for 260ms before next frame according to the edt protocol
        motor->beep(TONE5, MotorIf::VOLUME_OFF);
        break;
    case DSHOT_CMD_BEACON3:
        motor->beep(TONE3, MotorIf::VOLUME_LOW);
        timer->delay_ms(200); // FC will wait for 260ms before next frame according to the edt protocol
        motor->beep(TONE5, MotorIf::VOLUME_OFF);
        break;
    case DSHOT_CMD_BEACON4:
        motor->beep(TONE4, MotorIf::VOLUME_LOW);
        timer->delay_ms(200); // FC will wait for 260ms before next frame according to the edt protocol
        motor->beep(TONE5, MotorIf::VOLUME_OFF);
        break;
    case DSHOT_CMD_BEACON5:
        motor->beep(TONE5, MotorIf::VOLUME_LOW);
        timer->delay_ms(200); // FC will wait for 260ms before next frame according to the edt protocol
        motor->beep(TONE5, MotorIf::VOLUME_OFF);
        break;
    case DSHOT_CMD_ESC_INFO:
        send_value = 0;
        send_status = true;
        break; // V2 includes settings
    case DSHOT_CMD_SPIN_DIRECTION_NORMAL:
    case DSHOT_CMD_SPIN_DIRECTION_1:
        if (cmd_repeat >= 6)
            config.spin_dir_reverse = false;

        break;
    case DSHOT_CMD_SPIN_DIRECTION_REVERSED:
    case DSHOT_CMD_SPIN_DIRECTION_2:
        if (cmd_repeat >= 6)
            config.spin_dir_reverse = true;

        break;
    case DSHOT_CMD_3D_MODE_OFF:
        if (cmd_repeat >= 6)
            config.mode_3d = false;

        break;
    case DSHOT_CMD_3D_MODE_ON:
        if (cmd_repeat >= 6)
            config.mode_3d = true;
        break;
    case DSHOT_CMD_SETTINGS_REQUEST: // Currently not implemented by bf
        if (cmd_repeat >= 6)
            ;
        break;
    case DSHOT_CMD_SAVE_SETTINGS:
        if (cmd_repeat >= 6)
        {
            config.save();
        }
        break;
    case DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE:
        if (cmd_repeat >= 6 && !config.edt_mode)
        {
            config.edt_mode = true;
            send_value = 0;
            send_status = true;
        }
        break;
    case DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE:
        if (cmd_repeat >= 6 && config.edt_mode)
        {
            config.edt_mode = false;
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
        {
            throttle = value - 47;
            if (config.mode_3d)
            {
                if (throttle > 1000)
                    throttle = (throttle - 1000) * 2;
                else
                    throttle *= -2;
            }
            if (config.spin_dir_reverse)
                throttle = -throttle;
            motor->set_throttle(throttle);
        }
        break;
    }
}

void proccess(void)
{
    if (rd_sz > 0) // it won't rise at the end of the last pulse
        return;

    resp_time = now_us + 25; // 30us, 5us for error
    uint16_t dshot_bits = 0;

    if (__builtin_expect(freq_lock < 3, false))
    { // auto calc the period when disarmed
        uint32_t period;
        if (buffer[30] < buffer[0])
            period = (uint32_t)(buffer[30] + 65536 - buffer[0]) / 15;
        else
            period = (uint32_t)(buffer[30] - buffer[0]) / 15;
        half_period = period / 2;
        for (uint32_t i = 0; i < 32; i += 2)
        {
            dshot_bits <<= 1;
            register uint32_t low = buffer[i];
            register uint32_t high = buffer[i + 1];
            uint32_t pulse = high;
            if (high < low)
                pulse += 65536;
            pulse -= low;
            if (pulse > half_period)
                dshot_bits |= 1;
        }
        uint32_t tmp = period * 4 / 5;
        uint32_t delta = period45 > tmp ? period45 - tmp : tmp - period45;
        if (period45 && delta < 50)
            freq_lock++;
        else
            freq_lock = 0;
        period45 = tmp;
    }
    else
    {
        for (uint32_t i = 0; i < 32; i += 2)
        {
            dshot_bits <<= 1;
            register uint32_t low = buffer[i];
            register uint32_t high = buffer[i + 1];
            uint32_t pulse = high;
            if (__builtin_expect(high < low, false))
                pulse += 65536;
            pulse -= low;
            if (pulse > half_period)
                dshot_bits |= 1;
        }
    }

    uint32_t crc_sum = dshot_bits;
    crc_sum ^= crc_sum >> 8;
    crc_sum ^= crc_sum >> 4;
    crc_sum &= 0x0f;
    if (crc_sum == 0) // normal dshot
    {
        dshot_process(dshot_bits >> 4);
        frame_err = 0;
    }
    else if (crc_sum == 0xf) // dshot2d
    {
        dshot_process(dshot_bits >> 4);
        state = PREPARING;
        frame_err = 0;
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
    uint32_t encode_data = gcr_table[crc_sum];
    encode_data |= gcr_table[(data & 0xf)] << 5;
    encode_data |= gcr_table[(data & 0xf0) >> 4] << 10;
    encode_data |= gcr_table[(data & 0xf00) >> 8] << 15;
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

void fill_send_buffer(void)
{
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
    if (!config.edt_mode)
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

void send_prepare(void)
{
    if (send_status)
    {
        send_status = false;
        send_value = 0xE00 | send_value;
        fill_send_buffer();
        return;
    }

    if (!config.edt_mode)
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
        send_value = adc_volt->sample_voltage() / 250;
        send_value = 0x400 | (send_value * 4); // step 0.25v
    }
    else if (send_temparature)
    {
        send_temparature = false;
        send_value = adc_temp->sample_temperature() & 0xff;
        send_value = 0x200 | send_value; // step 1 Celsius degree
    }
    else
    {
        goto SEND_ERPM;
    }
    fill_send_buffer();
    return;

SEND_ERPM: // actually, the data sended is the period of erpm (1/erpm)
    // format e e e m m m m m m m m m,  erpm = M << E
    uint32_t E = 0;
    send_value = motor->get_e_period(); // 1000000/erpm us
    while (send_value > ((1 << 9) - 1))
    {
        send_value >>= 1;
        E++;
    }
    send_value = (E << 9) + send_value;

    fill_send_buffer();
}

void receive_dealing()
{
    proccess(); // one byte once to avoid long time cpu occupation
    int rd_sz = pwm->recv_pulses();
    if (rd_sz == sizeof(buffer) / sizeof(buffer[0]))
        return;
    if (rd_sz == ::rd_sz)
    { // no byte received over 20us, so restart frame
        restart();
        return;
    }
    ::rd_sz = rd_sz;
}

void send_dealing()
{
    if (now_us < resp_time)
        return;

    if (state == PREPARING)
    {
        send_prepare();
        state = PREPARED;
        return;
    }

    if (state == PREPARED)
    {
        pwm->send_pulses(buffer, 21, period45);
        state = SENDING;
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
        frame_err++;
        scheduling_telemetry();
        receive_dealing();
    }
}

bool Dshot::signal_lost()
{
    bool res = frame_err > 50000;
    if (res)
        motor->arm(false);
    return res;
}