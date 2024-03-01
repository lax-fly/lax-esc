#include "usart.h"
#include "gpio.h"
#include "assert.h"

#define ARRAY_CNT(x) (sizeof(x) / sizeof(x[0]))

usart_t usart_map[] = {
    {USART1,
     {PA9, GPIO_MUX_1, DMA1_CHANNEL2},
     {PA10, GPIO_MUX_1, DMA1_CHANNEL3}},
    {USART1,
     {PB6, GPIO_MUX_0, DMA1_CHANNEL2},
     {PB7, GPIO_MUX_0, DMA1_CHANNEL3}},

    {USART2,
     {PA2, GPIO_MUX_1, DMA1_CHANNEL4},
     {PA3, GPIO_MUX_1, DMA1_CHANNEL5}},
    {USART2,
     {PA8, GPIO_MUX_4, DMA1_CHANNEL4},
     {PB0, GPIO_MUX_3, DMA1_CHANNEL5}},
    {USART2,
     {PA14, GPIO_MUX_1, DMA1_CHANNEL4},
     {PA15, GPIO_MUX_1, DMA1_CHANNEL5}},
};

static volatile uint8_t rx_head, rx_tail;

static void dma_buffer_config(dma_channel_type *dmax_channely, uint32_t memory_base_addr, uint16_t buffer_size)
{
    dmax_channely->dtcnt = buffer_size;
    dmax_channely->maddr = memory_base_addr;
}

int dma_config(usart_t *usart)
{
    usart_type *usart_regs = usart->regs;
    dma_init_type dma_init_struct;
    dma_default_para_init(&dma_init_struct);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

    usart_dma_transmitter_enable(usart_regs, TRUE);
    usart_dma_receiver_enable(usart_regs, TRUE);

    dma_reset(usart->tx.dma_channel);
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_LOW;
    dma_init_struct.loop_mode_enable = FALSE;
    dma_init(usart->tx.dma_channel, &dma_init_struct);

    dma_reset(usart->rx.dma_channel);
    dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_init_struct.loop_mode_enable = TRUE;
    dma_init(usart->rx.dma_channel, &dma_init_struct);

    return 0;
}

enum UsartMode
{
    TX_ONLY,
    RX_ONLY,
    FULL_DUPLEX,
    HALF_DUPLEX
};

static void usart_config(usart_t *usart_cfg, uint32_t baud, UsartMode mode, float stopbit)
{
    usart_type *regs = usart_cfg->regs;

    switch ((uint32_t)regs)
    {
    case USART1_BASE:
        crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
        break;
    case USART2_BASE:
        crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK, TRUE);
        break;
    default:
        assert(false);
        break;
    }

    usart_stop_bit_num_type stop;
    if (stopbit < 1)
        stop = USART_STOP_0_5_BIT;
    else if (stopbit == 1)
        stop = USART_STOP_1_BIT;
    else if (stopbit < 2)
        stop = USART_STOP_1_5_BIT;
    else
        stop = USART_STOP_2_BIT;

    /* configure param */
    usart_init(regs, baud, USART_DATA_8BITS, stop);

    switch (mode)
    {
    case TX_ONLY: // tx only
        Gpio::setup_af(usart_cfg->tx.pin, Gpio::AF_OUTPUT_PP, usart_cfg->tx.af);
        usart_transmitter_enable(regs, TRUE);
        break;
    case RX_ONLY: // rx only
        usart_receiver_enable(regs, TRUE);
        Gpio::setup_af(usart_cfg->rx.pin, Gpio::AF_INPUT_PU, usart_cfg->rx.af);
        /* code */
        break;
    case FULL_DUPLEX: // full duplex
        Gpio::setup_af(usart_cfg->tx.pin, Gpio::AF_OUTPUT_PP, usart_cfg->tx.af);
        Gpio::setup_af(usart_cfg->rx.pin, Gpio::AF_INPUT_PU, usart_cfg->rx.af);
        usart_transmitter_enable(regs, TRUE);
        usart_receiver_enable(regs, TRUE);
        /* code */
        break;
    case HALF_DUPLEX:                                                            // half duplex
        Gpio::setup_af(usart_cfg->tx.pin, Gpio::AF_OUTPUT_OD, usart_cfg->tx.af); // need external pull up resistance
        usart_transmitter_enable(regs, TRUE);
        usart_receiver_enable(regs, TRUE);
        usart_single_line_halfduplex_select(regs, TRUE);
        break;

    default:
        break;
    }

    usart_parity_selection_config(regs, USART_PARITY_NONE);
    usart_interrupt_enable(regs, USART_RDBF_INT, TRUE);
    usart_enable(regs, TRUE);
}

UsartIf *UsartIf::new_instance(Pin tx_pin, Pin rx_pin, uint32_t baud, float stopbit)
{
    UsartMode mode = FULL_DUPLEX;
    assert(tx_pin != PIN_NONE || rx_pin != PIN_NONE);
    if (tx_pin == rx_pin)
        mode = HALF_DUPLEX;
    if (tx_pin == PIN_NONE && rx_pin != PIN_NONE)
        mode = RX_ONLY;
    if (rx_pin == PIN_NONE && tx_pin != PIN_NONE)
        mode = TX_ONLY;

    uint8_t iter;
    for (iter = 0; iter < ARRAY_CNT(usart_map); iter++)
    {
        if ((mode == HALF_DUPLEX || mode == TX_ONLY) && usart_map[iter].tx.pin == tx_pin)
            break;
        if (mode == RX_ONLY && usart_map[iter].rx.pin == rx_pin)
            break;
        if (mode == FULL_DUPLEX && usart_map[iter].tx.pin == tx_pin && usart_map[iter].rx.pin == rx_pin)
            break;
    }
    if (iter == ARRAY_CNT(usart_map))
        return nullptr;

    usart_config(&usart_map[iter], baud, mode, stopbit);
    dma_config(&usart_map[iter]);
    usart_map[iter].tx.dma_channel->paddr = (uint32_t)&usart_map[iter].regs->dt;
    usart_map[iter].rx.dma_channel->paddr = (uint32_t)&usart_map[iter].regs->dt;
    return new Usart(&usart_map[iter]);
}

int Usart::async_send(const uint8_t *data, int dsz)
{
    assert(dsz);
    dma_channel_type *dma = handle->tx.dma_channel;
    if (!data)
        return dsz - dma->dtcnt;
    dma->ctrl_bit.chen = 0;
    dma_buffer_config(dma, (uint32_t)data, dsz);
    dma->ctrl_bit.chen = 1;
    return 0;
}

void Usart::sync_send(const uint8_t *data, int dsz)
{
    async_send(data, dsz);
    while (dma_data_number_get(handle->tx.dma_channel) != 0)
    {
    } // wait for transmission
}

// return the data size already recieved by set buf = NULL
int Usart::async_recv(uint8_t *buf, int bsz)
{
    assert(bsz);

    dma_channel_type *dma = handle->rx.dma_channel;
    if (!buf)
        return bsz - dma->dtcnt;

    dma->ctrl_bit.chen = 0;
    dma_buffer_config(dma, (uint32_t)buf, bsz);
    dma->ctrl_bit.chen = 1;
    return 0;
}
