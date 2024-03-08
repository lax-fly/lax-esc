#pragma once

#include "stdint.h"

enum Angle
{                    // for bldc
    ANGLE_1_875 = 5, // degree 1.875
    ANGLE_3_75 = 4,  // degree 3.75
    ANGLE_7_5 = 3,   // degree 7.5
    ANGLE_15 = 2,    // degree 15
    ANGLE_30 = 1,    // degree 30
};

struct Config
{
    uint32_t version;
    
    bool edt_mode = false;         // Extended DShot Telemetry (EDT)
    bool serial_telemetry = false; // use serial telemetry?, on the bf configurator, you should enable esc sensor
    bool mode_3d;
    bool spin_dir_reverse;

    uint32_t polar_cnt;      // the polar count of the target motor
    uint32_t kv;             // rpm/v while empty load, used to judge current load
    uint32_t blind_interval; // us
    uint32_t startup_freq;   // Hz
    Angle commutate_angle;
    uint32_t bemf_threshold; // mV
    uint32_t max_pwm_freq;   // Hz
    uint32_t current_gain;   // the real current(mA) value divided by voltage(mV) from adc
    uint32_t voltage_gain;   // the read voltage value divided by voltage(mV) from adc

    // calibration value for shot type protocol
    uint32_t multishot_min;
    uint32_t multishot_max;
    uint32_t oneshot42_min;
    uint32_t oneshot42_max;
    uint32_t oneshot125_min;
    uint32_t oneshot125_max;
    uint32_t pwm_min;
    uint32_t pwm_max;

    void save();
    void load();
} __attribute__((aligned(4)));

extern Config config;