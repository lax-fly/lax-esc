#include "adc.h"
#include "gpio.h"
#include "msp.h"
#include <assert.h>

static bool g_adc_inited = false;
static uint32_t &adc_osq = *(uint32_t *)&ADC1->osq3;
static uint32_t &adc_ctrl2 = *(uint32_t *)&ADC1->ctrl2;
static volatile uint32_t &adc_sts = ADC1->sts;
static uint32_t &adc_odt = *(uint32_t *)&ADC1->odt;

static inline adc_channel_select_type pin2channel(Pin pin)
{
    return (adc_channel_select_type)GPIO2IDX(pin);
}

void Adc::prepare(void)
{
}

uint32_t Adc::sample_value(void) const
{
    adc_osq = ch; // change channel to this adc object
    adc_ctrl2 |= 1 << 22;
    while (!(adc_sts & ADC_CCE_FLAG))
    {
    }
    return adc_odt;
}

uint32_t Adc::sample_voltage(void) const
{
    return sample_value() * 3280 / 4096;
}

uint32_t Adc::sample_temperature(void) const
{
    return (sample_voltage() - 1280) * 10 / 43 + 25;
}

AdcIf *AdcIf::new_instance(Pin pin)
{
    assert(pin < PA8 || pin == PIN_MAX); // other pins are not supported yet

    adc_channel_select_type ch;
    if (pin == PIN_MAX)
        ch = ADC_CHANNEL_16;
    else
    {
        ch = pin2channel(pin);
        Gpio::setup_af(pin, Gpio::AF_ANALOG);
    }

    Adc *adc = new Adc(ch);
    if (g_adc_inited)
        return adc;

    g_adc_inited = true;
    adc_base_config_type adc_base_struct;
    crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
    crm_adc_clock_div_set(CRM_ADC_DIV_6);

    /* adc_settings----------------------------------------------------------- */
    adc_base_default_para_init(&adc_base_struct);
    adc_base_struct.sequence_mode = FALSE;
    adc_base_struct.repeat_mode = FALSE;
    adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
    adc_base_struct.ordinary_channel_length = 1;
    adc_base_config(ADC1, &adc_base_struct);

    /* adc_ordinary_conversionmode-------------------------------------------- */
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_0, 1, ADC_SAMPLETIME_1_5);
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_1, 1, ADC_SAMPLETIME_1_5);
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_2, 1, ADC_SAMPLETIME_1_5);
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_3, 1, ADC_SAMPLETIME_1_5);
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_4, 1, ADC_SAMPLETIME_1_5);
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_5, 1, ADC_SAMPLETIME_1_5);
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_6, 1, ADC_SAMPLETIME_1_5);
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_7, 1, ADC_SAMPLETIME_1_5);
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_16, 1, ADC_SAMPLETIME_1_5);

    adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_SOFTWARE, TRUE);

    adc_ordinary_part_mode_enable(ADC1, FALSE);

    adc_enable(ADC1, TRUE);

    /* adc calibration-------------------------------------------------------- */
    adc_calibration_init(ADC1);
    while (adc_calibration_init_status_get(ADC1))
    {
    }
    adc_calibration_start(ADC1);
    while (adc_calibration_status_get(ADC1))
    {
    }

    /* add user code begin adc1_init 2 */
    adc_flag_clear(ADC1, ADC_CCE_FLAG);
    /* add user code end adc1_init 2 */
    return adc;
}

uint16_t calc_avg(uint16_t *data, uint32_t sz)
{
    uint16_t max = 0;
    uint16_t min = 65535;
    uint16_t max_i = 0;
    uint16_t min_i = 0;
    for (uint32_t i = 0; i < sz; ++i)
    {
        if (max < data[i])
        {
            max = data[i];
            max_i = i;
        }
        if (min < data[i])
        {
            min = data[i];
            min_i = i;
        }
    }
    uint32_t avg = 0;
    for (uint32_t i = 0; i < sz; ++i)
    {
        if (i == min_i || i == max_i)
            continue;

        avg += data[i];
    }
    avg /= sz - 2;
    return (uint16_t)avg;
}
