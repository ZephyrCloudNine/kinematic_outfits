#include "config.h"
#include "pinouts.h"
#include "src/PRDC_ServoHT/PRDC_ServoHT.h"
#include "src/MillisTimer/MillisTimer.h"

// create servo object array
ServoHT servos[NUM_SERVOS];  
MillisTimer servos_handler_timer(SERVO_UPDATE_PERIOD_MS);
MillisTimer voltages_poll_timer(VOLTAGES_POLL_INT_MS);

uint32_t vbat_thresh_counter; 
bool low_battery = 0;

float vbat,vbus;

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
  LEDStartupIndicate();

  Serial.begin(115200);
  delay(STARTUP_DELAY_MS);
  Serial.print("----Starting KM controller:");

  #ifdef VARIANT_A

    Serial.println(" Device A----");

    //Set desired servo min-max limits
    servos[SERVO_1].setLimits(SERVO_1_START_POS,SERVO_1_END_POS);
    servos[SERVO_2].setLimits(SERVO_2_START_POS,SERVO_2_END_POS);
    servos[SERVO_3].setLimits(SERVO_3_START_POS,SERVO_3_END_POS);
    servos[SERVO_4].setLimits(SERVO_4_START_POS,SERVO_4_END_POS);
    servos[SERVO_5].setLimits(SERVO_5_START_POS,SERVO_5_END_POS);
    servos[SERVO_6].setLimits(SERVO_6_START_POS,SERVO_6_END_POS);
    servos[SERVO_7].setLimits(SERVO_7_START_POS,SERVO_7_END_POS);
    servos[SERVO_8].setLimits(SERVO_8_START_POS,SERVO_8_END_POS);

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
    servos[SERVO_1].write(SERVO_1_START_POS); 
    servos[SERVO_2].write(SERVO_2_START_POS); 
    servos[SERVO_3].write(SERVO_3_START_POS);      
    servos[SERVO_4].write(SERVO_4_START_POS); 
    servos[SERVO_5].write(SERVO_5_START_POS); 
    servos[SERVO_6].write(SERVO_6_START_POS); 
    servos[SERVO_7].write(SERVO_7_START_POS); 
    servos[SERVO_8].write(SERVO_8_START_POS);  

  #elif defined(VARIANT_B)
    Serial.println(" Device B----");

    //Set desired servo min-max limits
    servos[SERVO_1].setLimits(SERVO_1_START_POS,SERVO_1_END_POS);
    servos[SERVO_2].setLimits(SERVO_2_START_POS,SERVO_2_END_POS);
    servos[SERVO_3].setLimits(SERVO_3_START_POS,SERVO_3_END_POS);
    servos[SERVO_4].setLimits(SERVO_4_START_POS,SERVO_4_END_POS);

    //Set up servo instances
    servos[SERVO_1].attach(SV1_DOUT);
    servos[SERVO_2].attach(SV2_DOUT);
    servos[SERVO_3].attach(SV3_DOUT);
    servos[SERVO_4].attach(SV4_DOUT);

    //set servos to initial desired position
    servos[SERVO_1].write(SERVO_1_START_POS); 
    servos[SERVO_2].write(SERVO_2_START_POS); 
    servos[SERVO_3].write(SERVO_3_START_POS);      
    servos[SERVO_4].write(SERVO_4_START_POS); 

  #elif defined(VARIANT_C)
    Serial.println(" Device C----");

    //Set desired servo min-max limits
    servos[SERVO_8].setLimits(SERVO_8_START_POS,SERVO_8_END_POS);
    //Set up servo instances
    servos[SERVO_8].attach(SV8_DOUT);
    //set servos to initial desired position
    servos[SERVO_8].write(SERVO_8_START_POS); 

  #elif defined(VARIANT_D)
    Serial.println(" Device D----");

    //Set desired servo min-max limits
    servos[SERVO_8].setLimits(SERVO_8_START_POS,SERVO_8_END_POS);
    //Set up servo instances
    servos[SERVO_8].attach(SV8_DOUT);
    //set servos to initial desired position
    servos[SERVO_8].write(SERVO_8_START_POS); 
  #endif

  //Attach callback to soft timers 
  servos_handler_timer.expiredHandler(servoRunSequence);
  voltages_poll_timer.expiredHandler(readVoltages);
  //Start timers
  servos_handler_timer.start();
  voltages_poll_timer.start();

  //Sample voltages on startup
  readVoltages();
}

void loop()
{ 
  if (low_battery)
  {
    //Low battery condition - Halt loop and flash low battery indicator
    while (true)
    {
      digitalWrite(STAT2_LED,HIGH);
      delay(500);
      digitalWrite(STAT2_LED,LOW);
      delay(3000);
    }
  }
  //if VBUS voltage detected - USB cable is plugged in for charging. Halt loop and enter sleep mode
  if (vbus>VBUS_THRESH)
  {   
      //Flash LED to indicate charging has started
      digitalWrite(STAT1_LED,HIGH);
      delay(2500);
      digitalWrite(STAT1_LED,LOW);
      delay(1000);

      //Enter sleep mode
      while(true)
      {
        HAL_SuspendTick();
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);      
      }
  }

   //Run servo sequence
  servos_handler_timer.run();
  //Run voltages poll timer
  voltages_poll_timer.run();
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
void LEDStartupIndicate()
{
  for (uint8_t i=0;i<2;i++)
  {
    digitalWrite(STAT1_LED,HIGH);
    digitalWrite(STAT2_LED,LOW);
    delay(250);
    digitalWrite(STAT1_LED,LOW);
    digitalWrite(STAT2_LED,HIGH);
    delay(250);
  }

  digitalWrite(STAT1_LED,HIGH);
  digitalWrite(STAT2_LED,HIGH);
  delay(500); 
  digitalWrite(STAT1_LED,LOW);
  digitalWrite(STAT2_LED,LOW);
}

//Soft timer callback to handle updating servos
void servoRunSequence(MillisTimer &timer_handle)
{ 
  #ifdef VARIANT_A
    for (int i=SERVO_1;i<=SERVO_8;i++)
    { 
      if (!servos[i].readDir())
        servos[i].write(servos[i].read()+SERVO_STEP_DEGREES);
      else if (servos[i].readDir())
        servos[i].write(servos[i].read()-SERVO_STEP_DEGREES);
    }

  #elif defined(VARIANT_B)
    for (int i=SERVO_1;i<=SERVO_4;i++)
    { 
      if (!servos[i].readDir())
        servos[i].write(servos[i].read()+SERVO_STEP_DEGREES);
      else if (servos[i].readDir())
        servos[i].write(servos[i].read()-SERVO_STEP_DEGREES);
    }

  #elif defined(VARIANT_C)
    for (int i=SERVO_8;i<=SERVO_8;i++)
    { 
      if (!servos[i].readDir())
        servos[i].write(servos[i].read()+SERVO_STEP_DEGREES);
      else if (servos[i].readDir())
        servos[i].write(servos[i].read()-SERVO_STEP_DEGREES);
    }

  #elif defined(VARIANT_D)
    for (int i=SERVO_8;i<=SERVO_8;i++)
    { 
      if (!servos[i].readDir())
        servos[i].write(servos[i].read()+SERVO_STEP_DEGREES);
      else if (servos[i].readDir())
        servos[i].write(servos[i].read()-SERVO_STEP_DEGREES);
    }

  #endif
 
}

// function to monitor battery and USB line voltages
void readVoltages()
{
  vbat = readCorrectedVoltage(VBAT_MON);
  vbus = readCorrectedVoltage(VBUS_MON);

  if (vbat<VBAT_UVLO)
  //Check if VBAT is below low battery threshold
  {
    //increment counter
    vbat_thresh_counter += VOLTAGES_POLL_INT_MS;
  }
  else
  {
    //reset counter if voltage is above threshold
    vbat_thresh_counter = 0;
  }
  //Voltage drop is sustained - set low battery condition
  if (vbat_thresh_counter > VBAT_UVLO_TIME_THRESH_MS)
  {
    low_battery = 1;
  }

  //Print VBAT and VBUS voltages
  Serial.print("VBAT(V) :");
  Serial.println(vbat,2);

  Serial.print("VBUS(V) :");
  Serial.println(vbus,2);
  Serial.println();
}

//Callback wrapper function to periodically monitor battery and USB line voltages
void readVoltages(MillisTimer &mt)
{
  readVoltages();
}

