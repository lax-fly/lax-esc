#pragma once
#include "msp.h"
#include "protocol.h"

class Dshot : public Protocol
{
private:
    enum State
    {
        TO_SEND,
        SENDING,
        RECEIVE,
    };
    TimerIf *timer;
    PwmIf *pwm;
    Protocol::CallBack callback;
    uint64_t run_time;
    uint32_t rx_buf[32];
    uint32_t tx_buf[32];
    uint16_t dshot_bit;
    Package package;
    int rd_sz;
    State state;
    uint64_t resp_time;
    uint32_t period;
    uint64_t now_us;
    void restart(void);
    void proccess(void);
    void receive_dealing();
    void send_dealing();
    uint32_t encode2dshot_bits(uint32_t data);

public:
    Dshot();
    virtual ~Dshot();
    virtual void set_package_callback(Protocol::CallBack callback);
    virtual void send_package(const Package &pakcage);
    virtual void poll(void);
};