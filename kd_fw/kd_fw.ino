
#include "Arduino.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "FastLED.h"
#include "String.h"
#include "stdlib.h"

//initializing PCA9685 servo instances 
Adafruit_PWMServoDriver driver[NUM_DRIVERS] = {Adafruit_PWMServoDriver(D1_I2C_ADDR),Adafruit_PWMServoDriver(D2_I2C_ADDR)};
// Define the array of leds
CRGB leds[NUM_LEDS];

void setup()
{
    Serial.begin(SERIAL_SPEED);
    //initialize RGB LED and turn it off 
    FastLED.addLeds<NEOPIXEL, FASTLED_PIN>(leds, NUM_LEDS);
    leds[0] = CRGB::Black;
    FastLED.show();

    Wire.begin(SDA_PIN,SCL_PIN);

    driver[D1_ID].begin();
    driver[D1_ID].setOscillatorFrequency(PCA9685_OSC_FREQ);
    driver[D1_ID].setPWMFreq(SERVO_FREQ); 
    driver[D2_ID].begin();
    driver[D2_ID].setOscillatorFrequency(PCA9685_OSC_FREQ);
    driver[D2_ID].setPWMFreq(SERVO_FREQ); 

    Serial.println("For manual mode, enter params in the format(driver_id, servo_channel_id, pulse length)(eg: 0,2,145)");
    delay(1500);
  
}

void loop()
{
  // Drive each servo one in a back/front gradual sweep
  // Serial.println(servonum);
  // for (uint16_t pulselen = SERVO_1_MIN_PULSE; pulselen < SERVO_1_MAX_PULSE; pulselen++)
  // {
  //   driver[SERVO_1_DRIVER_ID].setPWM(SERVO_1_CHANNEL, 0, pulselen);
  //   driver[SERVO_2_DRIVER_ID].setPWM(SERVO_2_CHANNEL, 0, pulselen);
  //   driver[SERVO_3_DRIVER_ID].setPWM(SERVO_3_CHANNEL, 0, pulselen);
  //   driver[SERVO_4_DRIVER_ID].setPWM(SERVO_4_CHANNEL, 0, pulselen);
  //   delay(10);
  // }

  // delay(500);

  // for (uint16_t pulselen = SERVO_1_MAX_PULSE; pulselen > SERVO_1_MIN_PULSE; pulselen--)
  // {
  //   driver[SERVO_1_DRIVER_ID].setPWM(SERVO_1_CHANNEL, 0, pulselen);
  //   driver[SERVO_2_DRIVER_ID].setPWM(SERVO_2_CHANNEL, 0, pulselen);
  //   driver[SERVO_3_DRIVER_ID].setPWM(SERVO_3_CHANNEL, 0, pulselen);
  //   driver[SERVO_4_DRIVER_ID].setPWM(SERVO_4_CHANNEL, 0, pulselen);
  //   delay(10);
  // }

  // delay(500);

  //Manual control for trial and error -  recommended to do this for each servo to find minimum and max values
  manualCalibrateServo();
}

void manualCalibrateServo()
{
  if (Serial.available() > 0)
  {
      // Read the input string from the serial monitor
      String input = Serial.readStringUntil('\n');

      // Call the function to process the CSV string
      int startIndex = 0;
      int endIndex = input.indexOf(',');

      uint8_t driver_id;
      uint8_t channel_id;
      uint16_t pulse_len;

    while (endIndex != -1)
    {
      // Extract the substring from startIndex to endIndex
      String value = input.substring(startIndex, endIndex);

      if (startIndex == 0)
        driver_id = value.toInt();
      else if (startIndex == 2)
        channel_id = value.toInt();

      // Update startIndex to the character after the last found ','
      startIndex = endIndex + 1;

      // Find the next ','
      endIndex = input.indexOf(',', startIndex);
    } 

    // Process the last value after the last ','
    String value = input.substring(startIndex);
    pulse_len = value.toInt();

    Serial.print("Running ");
    Serial.print(driver_id);
    Serial.print("-");
    Serial.print(channel_id);
    Serial.print("-");
    Serial.print(pulse_len);
    Serial.println();
    
    driver[driver_id].setPWM(channel_id, 0, pulse_len);
  }
}


