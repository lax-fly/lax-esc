#pragma once

#include <stdint.h>
#include "msp.h"
#include "at32f421_usart.h"

typedef struct
{
    Pin pin;
    gpio_mux_sel_type af;
    dma_channel_type *dma_channel;
} usart_pin;

typedef struct
{
    usart_type *regs;
    usart_pin tx;
    usart_pin rx;

} usart_t;

class Usart : public UsartIf
{
private:
    usart_t *handle;

public:
    Usart(usart_t *handle) : handle(handle){};
    virtual int async_send(const uint8_t *data, int dsz);
    virtual void sync_send(const uint8_t *data, int dsz);
    virtual int async_recv(uint8_t *buf, int bsz);
};