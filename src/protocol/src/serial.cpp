#include "serial.h"
#include "string.h"
#include "stdlib.h"
#include "motor.h"

extern UsartIf *debug_usart;
extern MotorIf *motor;

Serial::Serial()
{
    timer = TimerIf::singleton();
    run_time = 0;
    restart();
}

Serial::~Serial()
{
}

void Serial::restart(void)
{
    state = SYNC;
    process_idx = rd_sz = 0;
    debug_usart->async_recv(rx_buf, sizeof(rx_buf));
}

#define STR_CMP(x, y) strncmp(x, y, sizeof(y) - 1)

void Serial::proccess(void)
{
    if (process_idx == rd_sz)
        return;

    process_idx++;

    if (process_idx > 2)
    {
        if (rx_buf[process_idx - 2] == '\r' && rx_buf[process_idx - 1] == '\n')
        {
            rx_buf[process_idx - 2] = 0;
            char *p = (char *)rx_buf;
            if (STR_CMP((char *)rx_buf, "throttle ") == 0)
            {
                p += 9;
            }
            int value = strtod(p, nullptr);
            motor->set_throttle(value / 2000.0f);
            restart();
        }
        else if (process_idx == sizeof(rx_buf) / sizeof(rx_buf[0]))
            restart();
    }
}

void Serial::poll(void)
{
    proccess(); // one byte once to avoid long time cpu occupation
    uint32_t now = timer->now_ms();
    if (now < run_time)
        return;
    run_time = now + 1; // baudrate > 9600 required
    int rd_sz = debug_usart->async_recv(nullptr, sizeof(rx_buf));
    if (rd_sz == 0)
        return;
    if (rd_sz == this->rd_sz && process_idx == rd_sz)
    { // no byte received over 20us, so restart frame
        restart();
        return;
    }
    this->rd_sz = rd_sz;
}