#pragma once
#include <stdint.h>

#define GPIO(PORT, NUM) (((PORT) - 'A') * GPIO_NUM_PINS + (NUM))
#define GPIO_NUM_PINS 16
#define GPIO2PORT(PIN) ((PIN) / GPIO_NUM_PINS)
#define GPIO2BIT(PIN) (1U << GPIO2IDX(PIN))
#define GPIO2IDX(PIN) ((PIN) % GPIO_NUM_PINS)

enum Pin
{
    PIN_NONE = -1,
    PA0 = GPIO('A', 0),
    PA1 = GPIO('A', 1),
    PA2 = GPIO('A', 2),
    PA3 = GPIO('A', 3),
    PA4 = GPIO('A', 4),
    PA5 = GPIO('A', 5),
    PA6 = GPIO('A', 6),
    PA7 = GPIO('A', 7),
    PA8 = GPIO('A', 8),
    PA9 = GPIO('A', 9),
    PA10 = GPIO('A', 10),
    PA11 = GPIO('A', 11),
    PA12 = GPIO('A', 12),
    PA13 = GPIO('A', 13),
    PA14 = GPIO('A', 14),
    PA15 = GPIO('A', 15),
    PB0 = GPIO('B', 0),
    PB1 = GPIO('B', 1),
    PB2 = GPIO('B', 2),
    PB3 = GPIO('B', 3),
    PB4 = GPIO('B', 4),
    PB5 = GPIO('B', 5),
    PB6 = GPIO('B', 6),
    PB7 = GPIO('B', 7),
    // add more pin def according to your mcu
    PIN_MAX
};

#define DUTY_CYCLE(x) ((uint32_t)(x * 2000))

// interfaces definition
class MotorPwmIf
{ // for motor 3 pwm phase driver
public:
    virtual ~MotorPwmIf() {}
    virtual void set_dutycycle(uint32_t dutycycle) = 0; // 0-2000 map to 0.0~1.0
    /**
     * @brief set frequency, valid range: 1Hz~10MHz
     * when in pwm INPUT mode, this can set the measuring pulse range(max pulse 1/freq),
     * the larger range, the lower accuracy, whatever, the accuracy should be less than 1/freq/4000
     */
    virtual void set_freq(uint32_t freq) = 0;
    virtual void select(Pin pin) = 0;
    virtual uint32_t get_duty() const = 0;  // return the pwm duty length, unit insensitive(normally the timer tick count)
    virtual uint32_t get_cycle() const = 0; // return the pwm cycle length, unit insensitive(normally the timer tick count)
    virtual uint32_t get_pos() const = 0;   // return the current pwm output position in the cycle, unit insensitive(normally the timer tick count)
    static MotorPwmIf *new_instance(Pin a, Pin b, Pin c);
};

class SignalPwmIf
{ // for signal input detecting
public:
    enum Mode
    {
        // requirements by all mode: output resolution > 2000, input measuring resolution >= 65535, range > 2.5ms
        UP_PULSE_CAPTURE,     // only capture up pulses, and dosen't support output.
        PULSE_OUTPUT_CAPTURE, // output pulses and capture pulses，out put polarity is low, capture both up pulse and down pulse.
    };
    virtual ~SignalPwmIf() {}
    // set the PWM work mode, time insensitive, you can do many jobs in this function without worrying about time.
    virtual void set_mode(Mode mode = PULSE_OUTPUT_CAPTURE) = 0;

    /****************************** functions for PULSE_OUTPUT_CAPTURE mode only ******************************/
    /**
     * @brief write a array of pulses in ticks to the output, set pulses to null to check if last write is finished
     * if last write is finished, return 0, or return -1. send_pulses is asynchronous, works with set_freq function
     * @param pulses ouput pulses width in ticks, unit insensitive. but should be identical to that of recv_pulses
     * will be copied internally, unit ns, the value range should be support up to 500000ns.
     * when pulses=null, the function should return the data count left to be sent after last call with pulses!=0.
     * @note send_pulses function is not required to support multiple pins when using the same timer. let me put this in another way:
     * many mcu use timer peripheral to generate/capture pwm, and one timer support at most 4 channel(pins), but for efficiency,
     * one timer should be attached to only one pwm object when send_pulses is ever used, so the implementation runs faster without checking different channels.
     * in the meantime, the esc application will use only one pin to call send_pulses function
     * @return data count left to be sent
     */
    virtual int send_pulses(const uint16_t *pulses = 0, uint32_t sz = 0, uint32_t period = 0) = 0;
    /**
     * @brief measures the input pulses width in ticks, and save them in buffer 'pulses' which is unit insensitive.
     * recv_pulses measures both edge, so |```|___| is treated as two pulses(up pulse and down pulse)，
     * notice that recv_pulses doesn't care about polarity, in other words, the 'pulses' dosen't indicate the direction but only the pulse width.
     * when pulses=null, the function should return the data count received after last call with pulses!=0.
     * @return currently received pulses
     * @note reffer to send_pulses function
     */
    virtual int recv_pulses(uint16_t *pulses = 0, uint32_t sz = 0) = 0;
    /****************************** functions for PULSE_OUTPUT_CAPTURE mode only ******************************/

    /****************************** functions for UP_PULSE_CAPTURE mode only ******************************/
    typedef void (*Callback)(uint32_t pulse_width);
    /**
     * @brief capture up pulse, and call back when it's done
     */
    virtual void set_up_pulse_callback(Callback cb) = 0;
    /****************************** functions for UP_PULSE_CAPTURE mode only ******************************/

    static SignalPwmIf *new_instance(Pin pin);
};

class MotorIoIf
{
public:
    virtual ~MotorIoIf() {}
    // select 'pin' to output high and clear other two pins, if pin is none of 'a' 'b' 'c', then all pins will be cleared, if pin = PIN_MAX, then all a,b,c will be set.
    virtual void select(Pin pin) = 0;
    static MotorIoIf *new_instance(Pin a, Pin b, Pin c);
};

class GpioIf
{ // used for debug only for now
public:
    enum IoMode
    {
        INPUT_FLOAT,
        INPUT_PULLUP,
        INPUT_PULLDOWN,
        OUTPUT,
        INPUT_OUTPUT,
    };
    virtual ~GpioIf() {}
    virtual void write(uint8_t bit) = 0;
    virtual uint8_t read() = 0;
    virtual void set() = 0;
    virtual void unset() = 0;
    virtual void toggle() = 0;
    virtual int set_mode(IoMode mode) = 0; // return 0 means ok while -1 means the mode is not supported by low level driver
    static GpioIf *new_instance(Pin pin);
};

class ComparatorIf
{
public:
    virtual ~ComparatorIf() {}
    // do some prepare work if necessary, such as changing the pins to current cmp object when multiple pins share the same cmp peripheral internally
    virtual void prepare() = 0;
    // make sure the cmp's output delay is below 1us and hysteresis below 10mV, or this driver may not work properly
    virtual uint8_t cmp_result() const = 0;
    // out_pin is not necessary, and used for debug, set out_pin to PIN_NONE when not used;
    static ComparatorIf *new_instance(Pin pos_pin, Pin neg_pin, Pin out_pin);
    static void set_callback(void (*cb)());
};

class AdcIf
{
public:
    virtual ~AdcIf() {}
    // do some prepare work if necessary, such as changing the pins to current adc object when multiple pins share the same adc peripheral internally
    virtual void prepare() = 0;
    virtual uint32_t sample_voltage(void) const = 0;     // in mV, // the implementing code must be done in 2us, which means the sample rate should be higher than 500kHz
    virtual uint32_t sample_temperature(void) const = 0; // in Celsius degree
    static AdcIf *new_instance(Pin pin);                 // when pin == PIN_MAX, this will return a instance binded to temperature channel
};

class TimerIf
{
public:
    typedef void (*Task)(void *);
    virtual void delay_us(uint16_t nus) = 0;
    virtual void delay_ms(uint16_t nms) = 0;
    virtual uint32_t now_ms(void) const = 0;                             // return current time in ms
    virtual uint64_t now_us(void) const = 0;                             // returncurrent time in us
    virtual void timing_task_1ms(Task task, void *data) = 0;             // support only one task, so caller make sure that the last task is finished before next calling
    virtual void delay_task_us(uint32_t nus, Task task, void *data) = 0; // support only one task, so caller make sure that the last task is finished before next calling
    static TimerIf *singleton();                                         // do not try to delete the returned object
};

class UsartIf
{
public:
    // return bytes count already sended by setting data = null
    virtual int async_send(const uint8_t *data, int dsz) = 0;
    virtual void sync_send(const uint8_t *data, int dsz) = 0;
    // return bytes count already received by setting buf = null
    virtual int async_recv(uint8_t *buf, int bsz) = 0;
    static UsartIf *new_instance(Pin tx_pin, Pin rx_pin, uint32_t baud, float stopbit);
};

class CrcIf // crc8
{           // not used yet
public:
    virtual void set_start(uint8_t val) = 0;
    virtual void set_poly(uint8_t poly) = 0;
    // calculate the crc of an array, reset automatically with each calling
    virtual uint8_t calc(uint8_t *data, uint32_t sz) = 0;
    // based on last remainder, reset by calling set_start
    virtual uint32_t calc(uint8_t data) = 0;
    static CrcIf *singleton(); // do not try to delete the returned object
};

void system_init(void);
