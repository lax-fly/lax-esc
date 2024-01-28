
#include "protocol.h"
#include "msp.h"

class Serial : public Protocol
{
private:
    enum State
    {
        SYNC,
        CMD,
        DATA_LENGTH,
        DATA,
        CRC,
    };
    TimerIf *timer;
    PwmIf *pwm;
    Protocol::CallBack callback;
    uint32_t run_time;
    uint8_t rx_buf[64];
    Package package;
    int state;
    int process_idx;
    int rd_sz;
    CrcIf *crc;
    uint8_t crc_sum;
    void restart(void);
    void proccess(void);

public:
    Serial();
    virtual ~Serial();
    virtual void set_package_callback(Protocol::CallBack callback);
    virtual void poll(void);
};