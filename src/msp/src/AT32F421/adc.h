#pragma once

#include "at32f421_adc.h"
#include "msp.h"

#define ADC_CHANNEL_A ADC_CHANNEL_5
#define ADC_CHANNEL_B ADC_CHANNEL_1
#define ADC_CHANNEL_C ADC_CHANNEL_0
#define ADC_CHANNEL_O ADC_CHANNEL_2
#define ADC_CHANNEL_VBAT ADC_CHANNEL_6
#define ADC_CHANNEL_VCUR ADC_CHANNEL_4

void init_adc_for_motor(void);

static inline void set_adc_sample_channel(int adc_channel)
{
    ADC1->osq3 = (adc_channel_select_type)adc_channel;
}

static inline uint32_t adc_sample(void)
{
    ADC1->ctrl2_bit.ocswtrg = 1;
    while (!(ADC1->sts & ADC_CCE_FLAG))
    {
    }
    return ADC1->odt_bit.odt;
}

extern float vbat;
extern uint16_t vbat_cnt;
void sample_vbat(void);

class Adc : public AdcIf
{
private:
    adc_channel_select_type ch;
    uint32_t sample_value(void) const;

public:
    Adc(adc_channel_select_type ch) : ch(ch) {}
    virtual uint32_t sample_voltage(void) const;
};
