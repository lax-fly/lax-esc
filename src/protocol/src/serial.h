
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
    uint32_t run_time;
    uint8_t rx_buf[64];
    int state;
    int process_idx;
    int rd_sz;
    uint8_t crc_sum;
    void restart(void);
    void proccess(void);

public:
    Serial();
    virtual ~Serial();
    virtual void poll(void);
    virtual bool signal_lost();
};