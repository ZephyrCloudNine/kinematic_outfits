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

// // I2C pin selection
// #define SDA_PIN 18
// #define SCL_PIN 19
// //Neopixel pin
// #define FASTLED_PIN 8

// //TTP223 Capacitive sensor module GPIO
// #define TOUCH_PIN 10

// #define SERIAL_SPEED 115200

// //defaulty frequency of PCA9865 is 27MHz
// #define PCA9685_OSC_FREQ 27000000
// //I2C address of each driver
// #define D1_I2C_ADDR 0x41
// #define D2_I2C_ADDR 0x40

// #define D1_ID 0
// #define D2_ID 1

// // Analog servos run at ~50 Hz updates
// //servo update interval should ideally be multiple of 50Hz/20ms (20ms,40ms,80ms)
// #define SERVO_FREQ 50 
// #define SERVO_UPDATE_PERIOD_MS 20

// #define NUM_DRIVERS 2
// #define NUM_SERVOS 15
// #define NUM_LEDS 1

// //servo starting hard stop limit (approximate for SG90 type)
// #define SERVO_ABS_SHL 120

// /* config of each servo's connected driver,channel, max and minimum pulse lengths - each servo may slightly differ
// in physical limits so allows to adjust movement for each servo individually. May need trial and error to exactly fine tune,
// pulse value is 12 bit(0-4096), SG90 servos can range between 100-450 absolute min-max */

// #define SERVO_1 0
// #define SERVO_1_DRIVER_ID 0
// #define SERVO_1_CHANNEL 0
// #define SERVO_1_MIN_PULSE 140
// #define SERVO_1_MAX_PULSE 230

// #define SERVO_2 1
// #define SERVO_2_DRIVER_ID 0
// #define SERVO_2_CHANNEL 1
// #define SERVO_2_MIN_PULSE 140
// #define SERVO_2_MAX_PULSE 230

// #define SERVO_3 2
// #define SERVO_3_DRIVER_ID 0
// #define SERVO_3_CHANNEL 3
// #define SERVO_3_MIN_PULSE 140
// #define SERVO_3_MAX_PULSE 230

// #define SERVO_4 3
// #define SERVO_4_DRIVER_ID 0
// #define SERVO_4_CHANNEL 4
// #define SERVO_4_MIN_PULSE 140
// #define SERVO_4_MAX_PULSE 230

// #define SERVO_5 4
// #define SERVO_5_DRIVER_ID 0
// #define SERVO_5_CHANNEL 5
// #define SERVO_5_MIN_PULSE 140
// #define SERVO_5_MAX_PULSE 230

// #define SERVO_6 5
// #define SERVO_6_DRIVER_ID 0
// #define SERVO_6_CHANNEL 7
// #define SERVO_6_MIN_PULSE 140
// #define SERVO_6_MAX_PULSE 300

// #define SERVO_7 6
// #define SERVO_7_DRIVER_ID 0
// #define SERVO_7_CHANNEL 8
// #define SERVO_7_MIN_PULSE 150
// #define SERVO_7_MAX_PULSE 300

// #define SERVO_8 7
// #define SERVO_8_DRIVER_ID 0
// #define SERVO_8_CHANNEL 9
// #define SERVO_8_MIN_PULSE 150
// #define SERVO_8_MAX_PULSE 300

// #define SERVO_9 8
// #define SERVO_9_DRIVER_ID 0
// #define SERVO_9_CHANNEL 11
// #define SERVO_9_MIN_PULSE 150
// #define SERVO_9_MAX_PULSE 300

// #define SERVO_10 9
// #define SERVO_10_DRIVER_ID 1
// #define SERVO_10_CHANNEL 8
// #define SERVO_10_MIN_PULSE 150
// #define SERVO_10_MAX_PULSE 300

// #define SERVO_11 10
// #define SERVO_11_DRIVER_ID 1
// #define SERVO_11_CHANNEL 1
// #define SERVO_11_MIN_PULSE 150
// #define SERVO_11_MAX_PULSE 280

// #define SERVO_12 11
// #define SERVO_12_DRIVER_ID 1
// #define SERVO_12_CHANNEL 3
// #define SERVO_12_MIN_PULSE 150
// #define SERVO_12_MAX_PULSE 280

// #define SERVO_13 12
// #define SERVO_13_DRIVER_ID 1
// #define SERVO_13_CHANNEL 4
// #define SERVO_13_MIN_PULSE 150
// #define SERVO_13_MAX_PULSE 280

// #define SERVO_14 13
// #define SERVO_14_DRIVER_ID 1
// #define SERVO_14_CHANNEL 5
// #define SERVO_14_MIN_PULSE 150
// #define SERVO_14_MAX_PULSE 280

// #define SERVO_15 14
// #define SERVO_15_DRIVER_ID 1
// #define SERVO_15_CHANNEL 7
// #define SERVO_15_MIN_PULSE 150
// #define SERVO_15_MAX_PULSE 280


#endif

