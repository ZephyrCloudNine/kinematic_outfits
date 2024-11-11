#ifndef CONFIG_H
#define CONFIG_h

#include "Arduino.h"

#define ADC_RESOLUTION_BITS 12
//Factor to multiply raw analogRead() to get voltage reading (V)
//Configured with setting resolution to 12 bits, so max value is 4096, and fixing max voltage to 3.3V
#define ADC_CONV_FACTOR 0.00081
// Resistor divider ratio on ADC pins (10/(5.1+10))k 
// Use this to get actual voltage value
#define DIVIDER_RATIO 0.662

//max number of servos supported by board
#define NUM_SERVOS 8

//servo indexes
#define SV1 0
#define SV2 1
#define SV3 2
#define SV4 3
#define SV5 4
#define SV6 5
#define SV7 6
#define SV8 7

#define VBAT_UVLO 3.50

// Analog servos run at ~50 Hz updates
//servo update interval should ideally be multiple of 50Hz/20ms (20ms,40ms,80ms)
#define SERVO_FREQ 50 
#define SERVO_UPDATE_PERIOD_MS 20

//servo starting hard stop limit (approximate for SG90 type)
#define SERVO_ABS_SHL 120

/* config of each servo's connected driver,channel, max and minimum pulse lengths - each servo may slightly differ
in physical limits so allows to adjust movement for each servo individually. May need trial and error to exactly fine tune,
pulse value is 12 bit(0-4096), SG90 servos can range between 100-450 absolute min-max */

#define SERVO_1 0
#define SERVO_1_DRIVER_ID 0
#define SERVO_1_CHANNEL 0
#define SERVO_1_MIN_PULSE 140
#define SERVO_1_MAX_PULSE 230

#define SERVO_2 1
#define SERVO_2_DRIVER_ID 0
#define SERVO_2_CHANNEL 1
#define SERVO_2_MIN_PULSE 140
#define SERVO_2_MAX_PULSE 230

#define SERVO_3 2
#define SERVO_3_DRIVER_ID 0
#define SERVO_3_CHANNEL 3
#define SERVO_3_MIN_PULSE 140
#define SERVO_3_MAX_PULSE 230

#define SERVO_4 3
#define SERVO_4_DRIVER_ID 0
#define SERVO_4_CHANNEL 4
#define SERVO_4_MIN_PULSE 140
#define SERVO_4_MAX_PULSE 230

#define SERVO_5 4
#define SERVO_5_DRIVER_ID 0
#define SERVO_5_CHANNEL 5
#define SERVO_5_MIN_PULSE 140
#define SERVO_5_MAX_PULSE 230

#define SERVO_6 5
#define SERVO_6_DRIVER_ID 0
#define SERVO_6_CHANNEL 7
#define SERVO_6_MIN_PULSE 140
#define SERVO_6_MAX_PULSE 300

#define SERVO_7 6
#define SERVO_7_DRIVER_ID 0
#define SERVO_7_CHANNEL 8
#define SERVO_7_MIN_PULSE 150
#define SERVO_7_MAX_PULSE 300

#define SERVO_8 7
#define SERVO_8_DRIVER_ID 0
#define SERVO_8_CHANNEL 9
#define SERVO_8_MIN_PULSE 150
#define SERVO_8_MAX_PULSE 300

#endif

