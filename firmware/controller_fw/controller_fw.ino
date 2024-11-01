#include "config.h"
#include "pinouts.h"

void setup()
{
  pinMode(STAT1_LED,OUTPUT);
  pinMode(STAT2_LED,OUTPUT);

  Serial.begin(115200);
  Serial.println("Starting KM controller");
  delay(1000);
}

void loop()
{
  Serial.println("DEBUGGING");
  digitalWrite(STAT1_LED,HIGH);
  delay(500);
  digitalWrite(STAT1_LED,LOW);
  digitalWrite(STAT2_LED,HIGH);
  delay(500);
  digitalWrite(STAT2_LED,LOW);
}