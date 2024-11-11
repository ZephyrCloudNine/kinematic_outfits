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
  delay(3000);
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

  //Flash on-board LED's
  // digitalWrite(STAT1_LED,!digitalRead(STAT1_LED));
  digitalWrite(STAT2_LED,!digitalRead(STAT2_LED));

  //Get servo to zero position
  servos[SV1].write(5); 
  
  // Check for battery voltage while running servos
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

float readCorrectedVoltage(uint32_t _pin)
{ 
  float temp_reading = analogRead(_pin);
  return ((temp_reading*(ADC_CONV_FACTOR/DIVIDER_RATIO)));
}