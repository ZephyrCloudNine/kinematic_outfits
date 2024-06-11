
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
uint32_t servo_update_interval[NUM_SERVOS];
bool servo_dir[NUM_SERVOS];
uint32_t servo_timer[NUM_SERVOS];

volatile bool sensor_trig = 0x00;
bool current_state = 0x00;
bool prev_state = 0x00;
bool run_system = 0x00;

void setup()
{
    Serial.begin(SERIAL_SPEED);
    Serial.println("---Voltaic dress mech controller v1.0---");

    // initialize RGB LED - flash color to indicate startup 
    FastLED.addLeds<NEOPIXEL, FASTLED_PIN>(leds, NUM_LEDS);
    leds[0] = CRGB::Green;
    FastLED.show();
    delay(1000);
    leds[0] = CRGB::Black;
    FastLED.show();

    //init touch sensor GPIO - no need pullups/pulldowns as module takes care of that 
    pinMode(TOUCH_PIN,INPUT);
    // attachInterrupt(TOUCH_PIN,sensorTrig,RISING);
    
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
      driver[0].setPWM(i, 0,SERVO_ABS_SHL);
      driver[1].setPWM(i,0,SERVO_ABS_SHL);
      servo_timer[i] = millis();
      servo_dir[i] = 0;
      servo_update_interval[i] = SERVO_UPDATE_PERIOD_MS;
    }
    
    //set servo starting positions
    servo_pulse_state[SERVO_1] = SERVO_1_MIN_PULSE;
    servo_pulse_state[SERVO_2] = SERVO_2_MAX_PULSE;
    servo_pulse_state[SERVO_3] = SERVO_3_MIN_PULSE;
    servo_pulse_state[SERVO_4] = SERVO_4_MAX_PULSE;
    servo_pulse_state[SERVO_5] = SERVO_5_MIN_PULSE;
    servo_pulse_state[SERVO_6] = SERVO_6_MAX_PULSE;
    servo_pulse_state[SERVO_7] = SERVO_7_MIN_PULSE;
    servo_pulse_state[SERVO_8] = SERVO_8_MIN_PULSE;
    servo_pulse_state[SERVO_9] = SERVO_9_MIN_PULSE;
    servo_pulse_state[SERVO_10] = SERVO_10_MAX_PULSE;
    servo_pulse_state[SERVO_11] = SERVO_11_MIN_PULSE;
    servo_pulse_state[SERVO_12] = SERVO_12_MAX_PULSE;
    servo_pulse_state[SERVO_13] = SERVO_13_MIN_PULSE;
    servo_pulse_state[SERVO_14] = SERVO_14_MAX_PULSE;
    servo_pulse_state[SERVO_15] = SERVO_15_MIN_PULSE;
 
    Serial.print("Succesfully initialized system!");
    // Serial.println("In manual mode, enter params in the format(driver_id, servo_channel_id, pulse length)(eg: 0,2,145)");
    delay(1500);
  
}

void loop()
{
  //Run servo sequence
  runSequence();

  //Manual control for trial and error -  recommended to do this for each servo to find minimum and max values
  // manualCalibrateServo();
}

void sensorTrig()
{
  sensor_trig ^= 0x01;
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
    driver[driver_id].setPWM(channel_id, 0, pulse_len);
  }
}

void asyncdriveServo(uint8_t servo_id,uint8_t driver_id, uint8_t channel_id, uint16_t pulse_lower_lim, uint16_t pulse_upper_lim,uint32_t update_period_ms)
{
  //update servo positions only at predefined interval
  if (millis()-servo_timer[servo_id] > update_period_ms)
  { 
    //drive PCA9685 channel
    driver[driver_id].setPWM(channel_id, 0, servo_pulse_state[servo_id]);

    //decrement or increment position based on current direction
    if (!servo_dir[servo_id])
      servo_pulse_state[servo_id]++;
    else if (servo_dir[servo_id])
      servo_pulse_state[servo_id]--;

    //reverse servo direction if lower or upper position thresholds are reached
    if ((servo_pulse_state[servo_id] > pulse_upper_lim || servo_pulse_state[servo_id] <= pulse_lower_lim))
      servo_dir[servo_id]^=0x01;

    // Serial.print(servo_id);
    // Serial.print(":");
    // Serial.println(servo_pulse_state[servo_id]);

    servo_timer[servo_id] = millis();
  }
}

void runSequence()
{
  if (true)
  {
    asyncdriveServo(SERVO_1,SERVO_1_DRIVER_ID,SERVO_1_CHANNEL,SERVO_1_MIN_PULSE,SERVO_1_MAX_PULSE,SERVO_UPDATE_PERIOD_MS);
    asyncdriveServo(SERVO_2,SERVO_2_DRIVER_ID,SERVO_2_CHANNEL,SERVO_2_MIN_PULSE,SERVO_2_MAX_PULSE,SERVO_UPDATE_PERIOD_MS);
    asyncdriveServo(SERVO_3,SERVO_3_DRIVER_ID,SERVO_3_CHANNEL,SERVO_3_MIN_PULSE,SERVO_3_MAX_PULSE,SERVO_UPDATE_PERIOD_MS);
    asyncdriveServo(SERVO_4,SERVO_4_DRIVER_ID,SERVO_4_CHANNEL,SERVO_4_MIN_PULSE,SERVO_4_MAX_PULSE,SERVO_UPDATE_PERIOD_MS);
    asyncdriveServo(SERVO_5,SERVO_5_DRIVER_ID,SERVO_5_CHANNEL,SERVO_5_MIN_PULSE,SERVO_5_MAX_PULSE,SERVO_UPDATE_PERIOD_MS);
    asyncdriveServo(SERVO_6,SERVO_6_DRIVER_ID,SERVO_6_CHANNEL,SERVO_6_MIN_PULSE,SERVO_6_MAX_PULSE,SERVO_UPDATE_PERIOD_MS*2);
    asyncdriveServo(SERVO_7,SERVO_7_DRIVER_ID,SERVO_7_CHANNEL,SERVO_7_MIN_PULSE,SERVO_7_MAX_PULSE,SERVO_UPDATE_PERIOD_MS);
    asyncdriveServo(SERVO_8,SERVO_8_DRIVER_ID,SERVO_8_CHANNEL,SERVO_8_MIN_PULSE,SERVO_8_MAX_PULSE,SERVO_UPDATE_PERIOD_MS);
    asyncdriveServo(SERVO_9,SERVO_9_DRIVER_ID,SERVO_9_CHANNEL,SERVO_9_MIN_PULSE,SERVO_9_MAX_PULSE,SERVO_UPDATE_PERIOD_MS);
    asyncdriveServo(SERVO_10,SERVO_10_DRIVER_ID,SERVO_10_CHANNEL,SERVO_10_MIN_PULSE,SERVO_10_MAX_PULSE,SERVO_UPDATE_PERIOD_MS*2);
    asyncdriveServo(SERVO_11,SERVO_11_DRIVER_ID,SERVO_11_CHANNEL,SERVO_11_MIN_PULSE,SERVO_11_MAX_PULSE,SERVO_UPDATE_PERIOD_MS);
    asyncdriveServo(SERVO_12,SERVO_12_DRIVER_ID,SERVO_12_CHANNEL,SERVO_12_MIN_PULSE,SERVO_12_MAX_PULSE,SERVO_UPDATE_PERIOD_MS);
    asyncdriveServo(SERVO_13,SERVO_13_DRIVER_ID,SERVO_13_CHANNEL,SERVO_13_MIN_PULSE,SERVO_13_MAX_PULSE,SERVO_UPDATE_PERIOD_MS);
    asyncdriveServo(SERVO_14,SERVO_14_DRIVER_ID,SERVO_14_CHANNEL,SERVO_14_MIN_PULSE,SERVO_14_MAX_PULSE,SERVO_UPDATE_PERIOD_MS);
    asyncdriveServo(SERVO_15,SERVO_15_DRIVER_ID,SERVO_15_CHANNEL,SERVO_15_MIN_PULSE,SERVO_15_MAX_PULSE,SERVO_UPDATE_PERIOD_MS);
  }

}