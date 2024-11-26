#ifndef CONFIG_H
#define CONFIG_h

#include "Arduino.h"

//uncomment the appropriate config based on target device variant
#define VARIANT_A 
// #define VARIANT_B
// #define VARIANT_C
// #define VARIANT_D

#define ADC_RESOLUTION_BITS 12
//Factor to multiply raw analogRead() to get voltage reading (V)
//Configured with setting resolution to 12 bits, so END value is 4096, and fixing END voltage to 3.3V
#define ADC_CONV_FACTOR 0.00081
// Resistor divider ratio on ADC pins (10/(5.1+10))k 
// Use this to get actual voltage value
#define DIVIDER_RATIO 0.662

//Number of analog samples to capture to derive average voltage reading
#define SAMPLE_COUNT 10

//Battery low voltage threshold
#define VBAT_UVLO 3.50
#define VOLTAGES_POLL_INT_MS 1000
#define VBAT_UVLO_TIME_THRESH_MS VOLTAGES_POLL_INT_MS * 2

//VBUS min threshold
#define VBUS_THRESH 4.80

//END number of servos supported by board
#define NUM_SERVOS 8
// Analog servos run at ~50 Hz updates
//servo update interval should ideally be multiple of 50Hz/20ms (20ms,40ms,80ms)
#define SERVO_FREQ 50 
#define SERVO_UPDATE_PERIOD_MS 20
#define SERVO_STEP_DEGREES 1

//Delay before starting operations - adjust as needed
#define STARTUP_DELAY_MS 2000

/*servo config, index, start and end positions - this is variant specific and can be customized*/
#ifdef VARIANT_A

    #define SERVO_1 0
    #define SERVO_1_START_POS 5
    #define SERVO_1_END_POS 90

    #define SERVO_2 1
    #define SERVO_2_START_POS 5
    #define SERVO_2_END_POS 90

    #define SERVO_3 2
    #define SERVO_3_START_POS 5
    #define SERVO_3_END_POS 90

    #define SERVO_4 3
    #define SERVO_4_START_POS 5
    #define SERVO_4_END_POS 90

    #define SERVO_5 4
    #define SERVO_5_START_POS 5
    #define SERVO_5_END_POS 90

    #define SERVO_6 5
    #define SERVO_6_START_POS 5
    #define SERVO_6_END_POS 90

    #define SERVO_7 6
    #define SERVO_7_START_POS 5
    #define SERVO_7_END_POS 90

    #define SERVO_8 7
    #define SERVO_8_START_POS 5
    #define SERVO_8_END_POS 90

#endif

#ifdef VARIANT_B

    #define SERVO_1 0
    #define SERVO_1_START_POS 5
    #define SERVO_1_END_POS 90

    #define SERVO_2 1
    #define SERVO_2_START_POS 5
    #define SERVO_2_END_POS 90

    #define SERVO_3 2
    #define SERVO_3_START_POS 5
    #define SERVO_3_END_POS 90

    #define SERVO_4 3
    #define SERVO_4_START_POS 5
    #define SERVO_4_END_POS 90
#endif

#ifdef VARIANT_C

    #define SERVO_1 0
    #define SERVO_1_START_POS 5
    #define SERVO_1_END_POS 90

#endif

#ifdef VARIANT_D

    #define SERVO_1 0
    #define SERVO_1_START_POS 5
    #define SERVO_1_END_POS 90

#endif
#endif

