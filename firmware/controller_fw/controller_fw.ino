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

  //Set ADC resolution
  analogReadResolution(ADC_RESOLUTION_BITS);

  //Flash LED's to indicate startup
  LEDstartupindicate();

  Serial.begin(115200);
  delay(STARTUP_DELAY_MS);
  Serial.println("----Starting KM controller----");

  //Set up servo instances
  servos[SERVO_1].attach(SV1_DOUT);
  servos[SERVO_2].attach(SV2_DOUT);
  servos[SERVO_3].attach(SV3_DOUT);
  servos[SERVO_4].attach(SV4_DOUT);
  servos[SERVO_5].attach(SV5_DOUT);
  servos[SERVO_6].attach(SV6_DOUT);
  servos[SERVO_7].attach(SV7_DOUT);
  servos[SERVO_8].attach(SV8_DOUT);

  //set servos to initial desired position
  servos[SERVO_1].write(SERVO_1_MIN_POS); 
  servos[SERVO_2].write(SERVO_2_MIN_POS); 
  servos[SERVO_3].write(SERVO_3_MIN_POS);      
  servos[SERVO_4].write(SERVO_4_MIN_POS); 
  servos[SERVO_5].write(SERVO_5_MIN_POS); 
  servos[SERVO_6].write(SERVO_6_MIN_POS); 
  servos[SERVO_7].write(SERVO_7_MIN_POS); 
  servos[SERVO_8].write(SERVO_8_MIN_POS);  
  
}

void loop()
{
  //Print VBAT and VBUS voltages
  Serial.print("VBAT(V) :");
  Serial.println(readCorrectedVoltage(VBAT_MON),2);

  Serial.print("VBUS(V) :");
  Serial.println(readCorrectedVoltage(VBUS_MON),2);
  Serial.println();
 
  //Check for battery voltage while running servos
  if (readCorrectedVoltage(VBAT_MON)>VBAT_UVLO)
  for (int pos = 20; pos <= 100; pos += 1) { 
    servos[SERVO_1].write(pos); 
    servos[SERVO_2].write(pos); 
    servos[SERVO_3].write(pos);      
    servos[SERVO_4].write(pos); 
    servos[SERVO_5].write(pos); 
    servos[SERVO_6].write(pos); 
    servos[SERVO_7].write(pos); 
    servos[SERVO_8].write(pos);    

    delay(15);                       
  }
  for (int pos = 100; pos >= 20; pos -= 1) {
    servos[SERVO_1].write(pos); 
    servos[SERVO_2].write(pos); 
    servos[SERVO_3].write(pos);      
    servos[SERVO_4].write(pos); 
    servos[SERVO_5].write(pos); 
    servos[SERVO_6].write(pos); 
    servos[SERVO_7].write(pos); 
    servos[SERVO_8].write(pos);               
    delay(15);                      
  }

    delay(500); 
}

//Function calculates average voltage reading from raw analog readings - taking into account divider ratios too
float readCorrectedVoltage(uint32_t _pin)
{ 
  float temp_reading,avg_volts;

  for (uint8_t i;i<SAMPLE_COUNT;i++)
  {
    temp_reading += analogRead(_pin);
  }

  avg_volts = (temp_reading/SAMPLE_COUNT)*(ADC_CONV_FACTOR/DIVIDER_RATIO);
  return avg_volts;
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

