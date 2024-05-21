
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  75 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 190 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

void setup() {
  Serial.begin(115200);
  Serial.println("8 channel Servo test!");
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(1500);
  Serial.print("Set pulse width value: ");
}

void loop() {
  // Drive each servo one in a back/front gradual sweep
  // Serial.println(servonum);
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(servonum, 0, pulselen);
    delay(10);
  }

  delay(2000);

  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(servonum, 0, pulselen);
    delay(10);
  }

  delay(2000);

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

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}