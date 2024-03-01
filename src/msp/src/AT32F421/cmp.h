#pragma once
#include "at32f421_cmp.h"
#include "msp.h"
#include "gpio.h"

#define CMP_PIN_A CMP_NON_INVERTING_PA5
#define CMP_PIN_B CMP_NON_INVERTING_PA1
#define CMP_PIN_C CMP_NON_INVERTING_PA0
#define CMP_PIN_O CMP_INVERTING_PA2

class Comparator : public ComparatorIf
{
private:
    int input_pins;

public:
    Comparator(Pin pos_pin, Pin neg_pin, Pin out_pin);
    virtual void prepare();
    virtual uint8_t cmp_result() const;
};