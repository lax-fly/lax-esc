#pragma once

#include "at32f421_adc.h"
#include "msp.h"

class Adc : public AdcIf
{
private:
    adc_channel_select_type ch;
    uint32_t sample_value(void) const;

public:
    Adc(adc_channel_select_type ch) : ch(ch) {}
    virtual void prepare();
    virtual uint32_t sample_voltage(void) const;
    virtual uint32_t sample_temperature(void) const;
};
