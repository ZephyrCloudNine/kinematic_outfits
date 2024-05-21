#ifndef CONFIG_H
#define CONFIG_h

#include "Arduino.h"

// I2C pin selection
#define SDA_PIN 18
#define SCL_PIN 19
//Neopixel pin
#define FASTLED_PIN 8

#define SERIAL_SPEED 115200

//defaulty frequency of PCA9865 is 27MHz
#define PCA9685_OSC_FREQ 27000000
//I2C address of each driver
#define D1_I2C_ADDR 0x40
#define D2_I2C_ADDR 0x41

#define D1_ID 0
#define D2_ID 1

// Analog servos run at ~50 Hz updates
#define SERVO_FREQ 50 

#define NUM_DRIVERS 2
#define NUM_SERVOS 4
#define NUM_LEDS 1

/* config of each servo's connected driver,channel, max and minimum pulse lengths - each servo may slightly differ
in physical limits so allows to adjust movement for each servo individually. May need trial and error to exactly fine tune,
pulse value is 12 bit(0-4096), SG90 servos can range between 100-450 absolute min-max */

#define SERVO_1_DRIVER_ID 0
#define SERVO_1_CHANNEL 0
#define SERVO_1_MIN_PULSE 120
#define SERVO_1_MAX_PULSE 250

#define SERVO_2_DRIVER_ID 0
#define SERVO_2_CHANNEL 1
#define SERVO_2_MIN_PULSE 120
#define SERVO_2_MAX_PULSE 250

#define SERVO_3_DRIVER_ID 0
#define SERVO_3_CHANNEL 2
#define SERVO_3_MIN_PULSE 120
#define SERVO_3_MAX_PULSE 250

#define SERVO_4_DRIVER_ID 0
#define SERVO_4_CHANNEL 3
#define SERVO_4_MIN_PULSE 120
#define SERVO_4_MAX_PULSE 250

#endif

