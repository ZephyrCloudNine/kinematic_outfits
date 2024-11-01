#include "config.h"
#include "pinouts.h"
#include <PRDC_ServoHT.h>

// create servo object to control a servo
ServoHT servos[NUM_SERVOS];  

void setup()
{
  //configure all output pins
  pinMode(STAT1_LED,OUTPUT);
  pinMode(STAT2_LED,OUTPUT);
  digitalWrite(STAT1_LED,LOW);

  pinMode(_5V_EN,OUTPUT);
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
  
  //Set ADC resolution
  analogReadResolution(ADC_RESOLUTION_BITS);

  //Set up servo instances
  servos[SV1].attach(SV1_DOUT);
  servos[SV3].attach(SV3_DOUT);
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
  // digitalWrite(STAT1_LED,HIGH);
  delay(500);
  // digitalWrite(STAT1_LED,LOW);
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

  //Check for battery voltage while running servos
  if (readCorrectedVoltage(VBAT_MON)>VBAT_UVLO)
  for (int pos = 20; pos <= 100; pos += 1) { 
    servos[SV1].write(pos); 
    servos[SV3].write(pos);               
    delay(15);                       
  }
  for (int pos = 100; pos >= 20; pos -= 1) {
    servos[SV1].write(pos); 
    servos[SV3].write(pos);                     
    delay(15);                      
  }
}

float readCorrectedVoltage(uint32_t _pin)
{ 
  float temp_reading = analogRead(_pin);
  return ((temp_reading*(ADC_CONV_FACTOR/DIVIDER_RATIO)));
}