#include "config.h"
#include "pinouts.h"

void setup()
{
  pinMode(STAT1_LED,OUTPUT);
  pinMode(STAT2_LED,OUTPUT);

  pinMode(_5V_EN,OUTPUT);
  //Enable 5V rail
  digitalWrite(_5V_EN,HIGH);

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

  delay(1000);
}

float readCorrectedVoltage(uint32_t _pin)
{ 
  float temp_reading = analogRead(_pin);
  return ((temp_reading*(ADC_CONV_FACTOR/DIVIDER_RATIO)));
}