
#define MOS_A_HIGH_PIN PA10
#define MOS_B_HIGH_PIN PA9
#define MOS_C_HIGH_PIN PA8

#define MOS_A_LOW_PIN PB1
#define MOS_B_LOW_PIN PB0
#define MOS_C_LOW_PIN PA7

#define CMP_A_POS_PIN PA5
#define CMP_B_POS_PIN PA1
#define CMP_C_POS_PIN PA0

#define CMP_A_NEG_PIN PA2
#define CMP_B_NEG_PIN PA2
#define CMP_C_NEG_PIN PA2

#ifndef NDEBUG
#define CMP_OUT_PIN PA6
#else
#define CMP_OUT_PIN PIN_NONE
#endif

#define ADC_A_PIN CMP_A_POS_PIN
#define ADC_B_PIN CMP_B_POS_PIN
#define ADC_C_PIN CMP_C_POS_PIN

#define ADC_BAT_PIN PA3
#define ADC_CUR_PIN PA4

#define SIGNAL_IN_PIN PB4
