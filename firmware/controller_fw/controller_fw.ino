#include "config.h"
#include "pinouts.h"

void setup()
{
  pinMode(STAT1_LED,OUTPUT);
  pinMode(STAT2_LED,OUTPUT);

  pinMode(_5V_EN,OUTPUT);
  //Enable 5V rail
  digitalWrite(_5V_EN,HIGH);

  pinMode(SV1_DOUT,OUTPUT);
  pinMode(SV2_DOUT,OUTPUT);
  pinMode(SV3_DOUT,OUTPUT);
  pinMode(SV4_DOUT,OUTPUT);
  pinMode(SV5_DOUT,OUTPUT);
  pinMode(SV6_DOUT,OUTPUT);
  pinMode(SV7_DOUT,OUTPUT);
  pinMode(SV8_DOUT,OUTPUT);

  Serial.begin(115200);
  Serial.println("Starting KM controller");
  delay(1000);

  analogReadResolution(ADC_RESOLUTION_BITS);
}

void loop()
{
  //Print VBAT and VBUS voltages
  Serial.print("VBAT(V) :");
  Serial.println(readCorrectedVoltage(VBAT_MON),2);

  Serial.print("VBUS(V) :");
  Serial.println(readCorrectedVoltage(VBUS_MON),2);
  Serial.println();

  //Flash on-board LED's
  digitalWrite(STAT1_LED,HIGH);
  delay(500);
  digitalWrite(STAT1_LED,LOW);
  digitalWrite(STAT2_LED,HIGH);
  delay(500);
  digitalWrite(STAT2_LED,LOW);

  //Toggle servo outputs
  // digitalWrite(SV1_DOUT,!digitalRead(SV1_DOUT));
  // digitalWrite(SV2_DOUT,!digitalRead(SV2_DOUT));
  // digitalWrite(SV3_DOUT,!digitalRead(SV3_DOUT));
  // digitalWrite(SV4_DOUT,!digitalRead(SV4_DOUT));
  // digitalWrite(SV5_DOUT,!digitalRead(SV5_DOUT));
  // digitalWrite(SV6_DOUT,!digitalRead(SV6_DOUT));
  // digitalWrite(SV7_DOUT,!digitalRead(SV7_DOUT));
  // digitalWrite(SV8_DOUT,!digitalRead(SV8_DOUT));

  //PWM write servo outputs
  analogWrite(SV1_DOUT,130);
  analogWrite(SV2_DOUT,130);
  analogWrite(SV3_DOUT,130);
  analogWrite(SV4_DOUT,130);
  analogWrite(SV5_DOUT,130);
  analogWrite(SV6_DOUT,130);
  analogWrite(SV7_DOUT,130);
  analogWrite(SV8_DOUT,130);

  delay(1500);
}

float readCorrectedVoltage(uint32_t _pin)
{ 
  float temp_reading = analogRead(_pin);
  return ((temp_reading*(ADC_CONV_FACTOR/DIVIDER_RATIO)));
}