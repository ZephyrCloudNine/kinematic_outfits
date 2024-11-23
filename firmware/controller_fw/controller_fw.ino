#include "config.h"
#include "pinouts.h"
#include <PRDC_ServoHT.h>

// create servo object array
ServoHT servos[NUM_SERVOS];  

void setup()
{
  //configure all output pins
  pinMode(STAT1_LED,OUTPUT);
  pinMode(STAT2_LED,OUTPUT);

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

  //Flash LED's to indicate startup
  LEDstartupindicate();

  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting KM controller");
  
  //Set ADC resolution
  analogReadResolution(ADC_RESOLUTION_BITS);

  //Set up servo instances
  servos[SV1].attach(SV1_DOUT);
  servos[SV2].attach(SV2_DOUT);
  servos[SV3].attach(SV3_DOUT);
  servos[SV4].attach(SV4_DOUT);
  servos[SV5].attach(SV5_DOUT);
  servos[SV6].attach(SV6_DOUT);
  servos[SV7].attach(SV7_DOUT);
  servos[SV8].attach(SV8_DOUT);
}

void loop()
{
  //Print VBAT and VBUS voltages
  Serial.print("VBAT(V) :");
  Serial.println(readCorrectedVoltage(VBAT_MON),2);

  Serial.print("VBUS(V) :");
  Serial.println(readCorrectedVoltage(VBUS_MON),2);
  Serial.println();

  // Get servo to zero position
  servos[SV1].write(5); 
  
  //Check for battery voltage while running servos
  if (readCorrectedVoltage(VBAT_MON)>VBAT_UVLO)
  for (int pos = 20; pos <= 100; pos += 1) { 
    // servos[SV1].write(pos); 
    servos[SV2].write(pos); 
    servos[SV3].write(pos);      
    servos[SV4].write(pos); 
    servos[SV5].write(pos); 
    servos[SV6].write(pos); 
    servos[SV7].write(pos); 
    servos[SV8].write(pos);    

    delay(15);                       
  }
  for (int pos = 100; pos >= 20; pos -= 1) {
    // servos[SV1].write(pos); 
    servos[SV2].write(pos); 
    servos[SV3].write(pos);      
    servos[SV4].write(pos); 
    servos[SV5].write(pos); 
    servos[SV6].write(pos); 
    servos[SV7].write(pos); 
    servos[SV8].write(pos);               
    delay(15);                      
  }
 
    delay(500); 
}

//Function converts raw readings to volts, taking resistor dividers into account as well
float readCorrectedVoltage(uint32_t _pin)
{ 
  float temp_reading = analogRead(_pin);
  return ((temp_reading*(ADC_CONV_FACTOR/DIVIDER_RATIO)));
}

//LED flash pattern on controller startup
void LEDstartupindicate()
{
  for (uint8_t i=0;i<2;i++)
  {
    digitalWrite(STAT1_LED,HIGH);
    digitalWrite(STAT2_LED,LOW);
    delay(500);
    digitalWrite(STAT1_LED,LOW);
    digitalWrite(STAT2_LED,HIGH);
    delay(500);
  }

  digitalWrite(STAT1_LED,HIGH);
  digitalWrite(STAT2_LED,HIGH);
  delay(500); 
  digitalWrite(STAT1_LED,LOW);
  digitalWrite(STAT2_LED,LOW);
}