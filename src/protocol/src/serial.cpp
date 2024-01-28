#include "serial.h"
#include "string.h"
#include "stdlib.h"

extern UsartIf *debug_usart;

Serial::Serial() : callback(nullptr), run_time(0)
{
    timer = TimerIf::singleton();
    PwmIf *pwm = PwmIf::new_instance(PB4);
    crc = CrcIf::singleton();
    crc->set_start(0xFF);
    crc->set_poly(0xB7);
    restart();
}

Serial::~Serial()
{
    if (pwm)
        delete pwm;
}

void Serial::set_package_callback(Protocol::CallBack callback)
{
    this->callback = callback;
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

    uint8_t byte = rx_buf[process_idx++];

    if (process_idx > 2)
    {
        if (rx_buf[process_idx - 2] == '\r' && rx_buf[process_idx - 1] == '\n')
        {
            rx_buf[process_idx - 2] = 0;
            char *p = (char *)rx_buf;
            package.cmd = NONE;
            if (STR_CMP((char *)rx_buf, "throttle ") == 0)
            {
                package.cmd = THROTTLE;
                p += 9;
            }
            else if (STR_CMP((char *)rx_buf, "beep ") == 0)
            {
                package.cmd = THROTTLE;
                p += 5;
            }
            else if (STR_CMP((char *)rx_buf, "dir ") == 0)
            {
                package.cmd = THROTTLE;
                p += 4;
            }
            if (package.cmd != NONE)
            {
                package.value = strtod(p, nullptr);
                if (callback)
                    callback(package);
                restart();
            }
        }
    }
    // switch (state)
    // {
    // case SYNC:
    //     if (byte == 0x00)
    //     {
    //         state = CMD;
    //         crc->set_start(0xFF);
    //     }
    //     break;
    // case CMD:
    //     switch (byte)
    //     {
    //     case Protocol::BEEP:
    //     case Protocol::VERSION:
    //     case Protocol::DIR:
    //     case Protocol::MODE_3D:
    //     case Protocol::SETTING:
    //     case Protocol::SAVE_SETTING:
    //     case Protocol::THROTTLE:
    //         package.cmd = (Protocol::CMD)byte;
    //         state = DATA;
    //         crc_sum = crc->calc(byte);
    //         break;

    //     default:
    //         state = SYNC;
    //         break;
    //     }
    //     break;
    // case DATA:
    //     switch (byte)
    //     {
    //     case Protocol::BEEP:
    //     case Protocol::DIR:
    //     case Protocol::MODE_3D:
    //         package.value = (int8_t)byte;
    //         state = CRC;
    //         crc_sum = crc->calc(byte);
    //         break;
    //     case Protocol::SETTING:
    //     case Protocol::SAVE_SETTING:
    //     case Protocol::VERSION:
    //         state = CRC;
    //         crc_sum = crc->calc(byte);
    //         break;

    //     default:
    //         state = SYNC;
    //         break;
    //     }
    //     break;
    // case CRC:
    //     //if (crc_sum == byte)
    //     {
    //         if (callback)
    //             callback(package);
    //         debug_usart->async_send(rx_buf, process_idx);
    //         restart();
    //     }
    //     state = SYNC;
    //     break;

    // default:
    //     break;
    // }
}

void Serial::poll(void)
{
    proccess(); // one byte once to avoid long time cpu occupation
    uint32_t now = timer->now_ms();
    if (now < run_time)
        return;
    run_time = now + 1; // baudrate > 9600
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