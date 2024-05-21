
#include "config.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "FastLED.h"

//initializing PCA9685 servo instances 
Adafruit_PWMServoDriver driver[NUM_DRIVERS] = {Adafruit_PWMServoDriver(D1_I2C_ADDR),Adafruit_PWMServoDriver(D2_I2C_ADDR)};
// Define the array of leds
CRGB leds[NUM_LEDS];

void setup()
{
    Serial.begin(SERIAL_SPEED);
    Serial.println("8 channel Servo test!");

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

    delay(1500);
    Serial.print("Set pulse width value: ");
}

void loop()
{
  // Drive each servo one in a back/front gradual sweep
  // Serial.println(servonum);
  for (uint16_t pulselen = SERVO_1_MIN_PULSE; pulselen < SERVO_1_MAX_PULSE; pulselen++)
  {
    driver[0].setPWM(0, 0, pulselen);
    driver[SERVO_2_DRIVER_ID].setPWM(SERVO_2_CHANNEL, 0, pulselen);
    driver[SERVO_3_DRIVER_ID].setPWM(SERVO_3_CHANNEL, 0, pulselen);
    driver[SERVO_4_DRIVER_ID].setPWM(SERVO_4_CHANNEL, 0, pulselen);
    delay(10);
  }

  delay(500);

  for (uint16_t pulselen = SERVO_1_MAX_PULSE; pulselen > SERVO_1_MIN_PULSE; pulselen--)
  {
    driver[0].setPWM(0, 0, pulselen);
    driver[SERVO_2_DRIVER_ID].setPWM(SERVO_2_CHANNEL, 0, pulselen);
    driver[SERVO_3_DRIVER_ID].setPWM(SERVO_3_CHANNEL, 0, pulselen);
    driver[SERVO_4_DRIVER_ID].setPWM(SERVO_4_CHANNEL, 0, pulselen);
    delay(10);
  }

  delay(500);

  //Manual control for trial and error -  recommended to do this for each servo to find minimum and max values
  // if (Serial.available() > 0) { // Check if data is available to read
  // uint16_t pulselen = Serial.parseInt(); // Read the integer value from the Serial port

  // // Print the received value back to the Serial monitor
  // Serial.print("Set pulse width: ");
  // Serial.println(pulselen);

  // pwm.setPWM(servonum, 0, pulselen);
  // Serial.println("Set pulse width value: ");
  // }

  // servonum++;
  // if (servonum > 0) servonum = 0; // Testing the first 8 servo channels
}
