#pragma once
#include <stdint.h>
#include "msp.h"

class Flash : public FlashIf
{
    virtual void write(uint8_t *data, uint32_t sz);
    virtual void read(uint8_t *data, uint32_t sz);
};
