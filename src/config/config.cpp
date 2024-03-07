#include "config.h"
#include "msp.h"

Config config;

void Config::save()
{
    FlashIf* flash = FlashIf::singleton();
    flash->write((uint8_t*)&config, sizeof(config));
}

void Config::load()
{
    FlashIf* flash = FlashIf::singleton();
    flash->read((uint8_t*)&config, sizeof(config));
    if (config.version == 0xaa550001)
        return;
    
    config.version = 0xaa550001;
    config.serial_telemetry = false;
    config.mode_3d = false;
    config.edt_mode = false;
    config.spin_dir_reverse = false;

    config.polar_cnt = 14;                   // the polar count of the target motor
    config.kv = 1400;                        // rpm/v while empty load, used to judge current load
    config.blind_interval = 20000;           // us
    config.startup_freq = 5000;              // Hz
    config.commutate_angle = ANGLE_30;
    config.bemf_threshold = 50;  // mV
    config.max_pwm_freq = 50000; // Hz
    config.current_gain = 5;     // the real current(mA) value divided by voltage(mV) from adc
    config.voltage_gain = 11;    // the read voltage value divided by voltage(mV) from adc
}