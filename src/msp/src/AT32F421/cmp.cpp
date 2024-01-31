#include "CMP.h"
#include "gpio.h"
#include "assert.h"

static bool cmp1_inited = false;

ComparatorIf *ComparatorIf::new_instance(Pin pos_pin, Pin neg_pin, Pin out_pin)
{
    assert(pos_pin == PA0 || pos_pin == PA1 || pos_pin == PA5);
    assert(neg_pin == PA0 || neg_pin == PA2 || neg_pin == PA4 || neg_pin == PA5);
    Gpio::setup_af(pos_pin, Gpio::AF_ANALOG);
    Gpio::setup_af(neg_pin, Gpio::AF_ANALOG);
    Comparator *cmp = new Comparator(pos_pin, neg_pin, out_pin);
    if (cmp1_inited)
        return cmp;
    cmp1_inited = true;
    cmp_init_type cmp_init_struct;
    crm_periph_clock_enable(CRM_CMP_PERIPH_CLOCK, TRUE);
    cmp_default_para_init(&cmp_init_struct);
    cmp_init_struct.cmp_non_inverting = CMP_NON_INVERTING_PA5;
    cmp_init_struct.cmp_inverting = CMP_INVERTING_PA2;
    cmp_init_struct.cmp_output = CMP_OUTPUT_NONE;
    cmp_init_struct.cmp_polarity = CMP_POL_NON_INVERTING;
    cmp_init_struct.cmp_speed = CMP_SPEED_FAST;
    cmp_init_struct.cmp_hysteresis = CMP_HYSTERESIS_NONE;
    cmp_init(CMP1_SELECTION, &cmp_init_struct);
    cmp_enable(CMP1_SELECTION, TRUE);
    cmp_scal_brg_config(CMP_SCAL_BRG_11);
    cmp_filter_config(0x3f, 0x3f, TRUE);
    return cmp;
}

Comparator::Comparator(Pin pos_pin, Pin neg_pin, Pin out_pin)
{
    switch (pos_pin)
    {
    case PA5:
        non_invert_in = CMP_NON_INVERTING_PA5;
        break;
    case PA1:
        non_invert_in = CMP_NON_INVERTING_PA1;
        break;
    case PA0:
        non_invert_in = CMP_NON_INVERTING_PA0;
        break;
    default:
        assert(false);
        break;
    }
    switch (neg_pin)
    {
    case PA4:
        invert_in = CMP_INVERTING_PA4;
        break;
    case PA5:
        invert_in = CMP_INVERTING_PA5;
        break;
    case PA0:
        invert_in = CMP_INVERTING_PA0;
        break;
    case PA2:
        invert_in = CMP_INVERTING_PA2;
        break;
    default:
        assert(false);
        break;
    }
    switch (out_pin)
    {
    case PA0:
        Gpio::setup_af(out_pin, Gpio::AF_OUTPUT_PP, GPIO_MUX_7);
        break;
    case PA6:
        Gpio::setup_af(out_pin, Gpio::AF_OUTPUT_PP, GPIO_MUX_7);
        break;
    case PIN_NONE:
        break;
    default:
        assert(false);
        break;
    }
}

uint8_t Comparator::cmp_result() const
{
    CMP->ctrlsts_bit.cmpninvsel = non_invert_in;
    CMP->ctrlsts_bit.cmpinvsel = invert_in;
    return CMP->ctrlsts_bit.cmpvalue;
}
