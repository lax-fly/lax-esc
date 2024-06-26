#include "config.h"
#include "msp.h"
#include "stdio.h"

Config config;
#define CONFIG_VERSION 0xaa550001

void Config::save()
{
    FlashIf *flash = FlashIf::singleton();
    flash->write((uint8_t *)&config, sizeof(config));
}

void print_config(void)
{
    const char *degree = "";
    switch (config.commutate_angle)
    {
    case ANGLE_30:
        degree = "30";
        break;
    case ANGLE_15:
        degree = "15";
        break;
    case ANGLE_7_5:
        degree = "7.5";
        break;
    case ANGLE_3_75:
        degree = "3.75";
        break;
    case ANGLE_1_875:
        degree = "1.875";
        break;

    default:
        break;
    }
    // \r means using sync print, without \r starting, async print is used
    printf("\r################### configuration start ####################\n");
    printf("\r  version:                 %7lu\n", config.version);
    printf("\r  serial telemetry:        %7s\n", config.serial_telemetry ? "on" : "off");
    printf("\r  3d mode:                 %7s\n", config.mode_3d ? "on" : "off");
    printf("\r  EDT mode:                %7s\n", config.edt_mode ? "on" : "off");
    printf("\r  spin reversed:           %7s\n", config.spin_dir_reverse ? "on" : "off");
    printf("\r  polar count:             %7lu\n", config.polar_cnt);
    printf("\r  motor kv:                %7lu\n", config.kv);
    printf("\r  blind comutate interval: %7lu us\n", config.blind_interval);
    printf("\r  startup frequence:       %7lu\n", config.startup_freq);
    printf("\r  commutate angle:         %7s'\n", degree);
    printf("\r  bemf threshold:          %7lu mV\n", config.bemf_threshold);
    printf("\r  max pwm frequence:       %7lu\n", config.max_pwm_freq);
    printf("\r  current gain:            %7lu\n", config.current_gain);
    printf("\r  voltage gain:            %7lu\n", config.voltage_gain);
    printf("\r  multishot min:           %7lu ns\n", config.multishot_min);
    printf("\r  multishot max:           %7lu ns\n", config.multishot_max);
    printf("\r  oneshot42 min:           %7lu ns\n", config.oneshot42_min);
    printf("\r  oneshot84 max:           %7lu ns\n", config.oneshot42_max);
    printf("\r  oneshot125 min:          %7lu ns\n", config.oneshot125_min);
    printf("\r  oneshot250 max:          %7lu ns\n", config.oneshot125_max);
    printf("\r  pwm max:                 %7lu ns\n", config.pwm_min);
    printf("\r  pwm min:                 %7lu ns\n", config.pwm_max);
    printf("\r################### configuration end  ####################\n");
}

void Config::load()
{
    FlashIf *flash = FlashIf::singleton();
    flash->read((uint8_t *)&config, sizeof(config));
    if (config.version == CONFIG_VERSION)
    {
        print_config();
        return;
    }

    config.version = CONFIG_VERSION;
    config.serial_telemetry = false;
    config.mode_3d = false;
    config.edt_mode = false;
    config.spin_dir_reverse = false;

    config.polar_cnt = 14;         // the polar count of the target motor
    config.kv = 1400;              // rpm/v while empty load, used to judge current load
    config.blind_interval = 20000; // us
    config.startup_freq = 5000;    // Hz
    config.commutate_angle = ANGLE_30;
    config.bemf_threshold = 50;  // mV
    config.max_pwm_freq = 50000; // Hz
    config.current_gain = 5;     // the real current(mA) value divided by voltage(mV) from adc
    config.voltage_gain = 11;    // the read voltage value divided by voltage(mV) from adc

    config.multishot_min = 5000;
    config.multishot_max = 25000;
    config.oneshot42_min = 42000;
    config.oneshot42_max = 84000;
    config.oneshot125_min = 125000;
    config.oneshot125_max = 250000;
    config.pwm_min = 1000000;
    config.pwm_max = 2000000;
    print_config();
}