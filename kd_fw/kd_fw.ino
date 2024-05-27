
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
uint16_t servo_pulse_state[NUM_SERVOS];
bool servo_dir[NUM_SERVOS];
uint32_t servo_timer[NUM_SERVOS];

void setup()
{
    Serial.begin(SERIAL_SPEED);
    //initialize RGB LED - flash color to indicate startup 
    FastLED.addLeds<NEOPIXEL, FASTLED_PIN>(leds, NUM_LEDS);
    leds[0] = CRGB::Green;
    FastLED.show();
    delay(1500);
    leds[0] = CRGB::Black;
    FastLED.show();
    
    Wire.begin(SDA_PIN,SCL_PIN);
    //initialize PCA9685 drivers
    driver[D1_ID].begin();
    driver[D1_ID].setOscillatorFrequency(PCA9685_OSC_FREQ);
    driver[D1_ID].setPWMFreq(SERVO_FREQ); 
    driver[D2_ID].begin();
    driver[D2_ID].setOscillatorFrequency(PCA9685_OSC_FREQ);
    driver[D2_ID].setPWMFreq(SERVO_FREQ); 

    for (uint8_t i=0;i<NUM_SERVOS;i++)
    {
      servo_pulse_state[i] = SERVO_ABS_SHL;
      servo_dir[i] = 0;
    }

    // Serial.println("For manual mode, enter params in the format(driver_id, servo_channel_id, pulse length)(eg: 0,2,145)");
    delay(1500);
  
}

void loop()
{
  //Run servo sequence
  runSequence();

  //Manual control for trial and error -  recommended to do this for each servo to find minimum and max values
  // manualCalibrateServo();
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
    
    //drive specified servo
    // driver[driver_id].setPWM(channel_id, 0, pulse_len);
  }
}

void asyncdriveServo(uint8_t servo_id,uint8_t driver_id, uint8_t channel_id, uint16_t pulse_start, uint16_t pulse_end)
{
  if (millis()-servo_timer[servo_id] > SERVO_UPDATE_INTERVAL_MS)
  {  
    if (servo_pulse_state[servo_id] < pulse_start)
    servo_pulse_state[servo_id] = pulse_start;
    driver[driver_id].setPWM(channel_id, 0, servo_pulse_state[servo_id]);

    if (!servo_dir[servo_id])
      servo_pulse_state[servo_id]++;
    else if (servo_dir[servo_id])
      servo_pulse_state[servo_id]--;

    if (servo_pulse_state[servo_id] >= pulse_end || servo_pulse_state[servo_id] <= pulse_start)
      servo_dir[servo_id]^=0x01;

    Serial.println(servo_pulse_state[servo_id]);
    
    servo_timer[servo_id] = millis();
  }
}

void runSequence()
{
  // asyncdriveServo(SERVO_1,SERVO_1_DRIVER_ID,SERVO_1_CHANNEL,SERVO_1_MIN_PULSE,SERVO_1_MAX_PULSE);
  // asyncdriveServo(SERVO_2,SERVO_2_DRIVER_ID,SERVO_2_CHANNEL,SERVO_2_MIN_PULSE,SERVO_2_MAX_PULSE);
  // asyncdriveServo(SERVO_3,SERVO_3_DRIVER_ID,SERVO_3_CHANNEL,SERVO_3_MIN_PULSE,SERVO_3_MAX_PULSE);
  // asyncdriveServo(SERVO_4,SERVO_4_DRIVER_ID,SERVO_4_CHANNEL,SERVO_4_MIN_PULSE,SERVO_4_MAX_PULSE);
  // asyncdriveServo(SERVO_5,SERVO_5_DRIVER_ID,SERVO_5_CHANNEL,SERVO_5_MIN_PULSE,SERVO_5_MAX_PULSE);
  // asyncdriveServo(SERVO_6,SERVO_6_DRIVER_ID,SERVO_6_CHANNEL,SERVO_6_MIN_PULSE,SERVO_6_MAX_PULSE);
  // asyncdriveServo(SERVO_7,SERVO_7_DRIVER_ID,SERVO_7_CHANNEL,SERVO_7_MIN_PULSE,SERVO_7_MAX_PULSE);


  for (uint16_t pl=SERVO_1_MIN_PULSE;pl<SERVO_1_MAX_PULSE;pl++)
  {
    driver[SERVO_1_DRIVER_ID].setPWM(SERVO_1_CHANNEL, 0, pl);
    driver[SERVO_2_DRIVER_ID].setPWM(SERVO_2_CHANNEL, 0, pl);
    driver[SERVO_3_DRIVER_ID].setPWM(SERVO_3_CHANNEL, 0, pl);
    driver[SERVO_4_DRIVER_ID].setPWM(SERVO_4_CHANNEL, 0, pl+125);
    driver[SERVO_5_DRIVER_ID].setPWM(SERVO_5_CHANNEL, 0, pl+50);
    driver[SERVO_6_DRIVER_ID].setPWM(SERVO_6_CHANNEL, 0, pl);
    driver[SERVO_7_DRIVER_ID].setPWM(SERVO_7_CHANNEL, 0, pl);
    delay(15);
  }
  
  for (uint16_t pl=SERVO_1_MAX_PULSE;pl>SERVO_1_MIN_PULSE;pl--)
  {
    driver[SERVO_1_DRIVER_ID].setPWM(SERVO_1_CHANNEL, 0, pl);
    driver[SERVO_2_DRIVER_ID].setPWM(SERVO_2_CHANNEL, 0, pl);
    driver[SERVO_2_DRIVER_ID].setPWM(SERVO_3_CHANNEL, 0, pl);
    driver[SERVO_3_DRIVER_ID].setPWM(SERVO_4_CHANNEL, 0, pl+125);
    driver[SERVO_4_DRIVER_ID].setPWM(SERVO_5_CHANNEL, 0, pl+50);
    driver[SERVO_5_DRIVER_ID].setPWM(SERVO_6_CHANNEL, 0, pl);
    driver[SERVO_7_DRIVER_ID].setPWM(SERVO_7_CHANNEL, 0, pl);
    delay(15);
  }
}