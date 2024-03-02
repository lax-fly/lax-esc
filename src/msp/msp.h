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

// interfaces definition

class PwmIf
{
public:
    virtual ~PwmIf() {}
    virtual void set_dutycycle(float dutycycle) = 0; // 0.0~1.0
    /**
     * @brief set frequency, valid range: 10Hz~100kHz
     * when in pwm INPUT mode, this is used to scale the measuring range, alse scales the value in data of recv_pulses and its accuracy
     * when in pwm OUTPUT mode, this is used to scale the output frequency range(to smaller), (1k~50k)/scale
     */
    virtual void set_freq(uint32_t freq) = 0;
    virtual void enable() = 0;
    virtual void disable() = 0;
    virtual uint32_t get_duty() const = 0;  // return the pwm duty length, unit insensitive(normally the timer tick count)
    virtual uint32_t get_cycle() const = 0; // return the pwm cycle length, unit insensitive(normally the timer tick count)
    virtual uint32_t get_pos() const = 0;   // return the current pwm output position in the cycle, unit insensitive(normally the timer tick count)
    /**
     *  @brief for output only, set the output polarity, edge > 0: high, edge <= 0: low
     */
    virtual void set_polarity(int edge = 1) = 0;
    /**
     *  @brief write a array of pulse in ns to the output, set pulses to null to check if last write is finished
     * if last write is finished, return 0, or return -1. send_pulses is asynchronous.
     *  @param pulses will be copied internally, unit ns, default support range: 0~500000ns.
     *  use set_freq to grow this range. note, the wider range, the lower accuracy
     *  for example, if you set_freq(1000), the period will be 1/1000 = 1ms, so the range will be 0~1ms
     */
    virtual int send_pulses(const uint32_t *pulses = 0, uint32_t sz = 0) = 0; //
    /**
     * @brief measuring the input pulses, |```|___| is treated as two pulses(up pulse and down pulse)，
     *        notice that the recv_pulses don't care about the pulse direction but the pulse width, pulse width must be in unit ns,
     *        the accuracy granularity must be lower than 1/freq/4000, where the 'freq' is set by set_freq
     * @return currently received pulses
     */
    virtual int recv_pulses(uint32_t *pulses = 0, uint32_t sz = 0) = 0;
    /**
     * @brief can capture
     * @return -1 if no pulse received, else return the most recently received pulse width in ns
     */
    virtual int recv_high_pulse() = 0;
    static PwmIf *new_instance(Pin pin);
};

class GpioIf
{
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
{
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
