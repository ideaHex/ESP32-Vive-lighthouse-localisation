// Board: ESP32 Dev Module
// https://github.com/nairol/LighthouseRedox/blob/master/docs/Light%20Emissions.md

// https://github.com/HiveTracker/HiveTracker.github.io
//https://github.com/ashtuchkin/vive-diy-position-sensor/wiki/Position-calculation-in-detail
// http://doc-ok.org/?p=1478
// https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/mcpwm.html

#include <driver/mcpwm.h>
#include "driver/timer.h"

#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#define CAP0_INT_EN BIT(27)  //Capture 0 interrupt bit
#define XAXIS 0
#define YAXIS 1

const uint32_t HEIGHT = 3000; // Height of light house above sensor

const int SensorPin[2] = {19, 23};
const uint32_t NumOfSensors = 2;
float answer;
int32_t X[NumOfSensors];
int32_t Y[NumOfSensors];
float yAngle[NumOfSensors];
float xAngle[NumOfSensors];
uint32_t interval[NumOfSensors];
float angle[NumOfSensors][2];

int syncAxis;
int syncData;
int syncSkip;


int waiting4Pulse;

uint32_t previous;
uint32_t temp;
bool flag = 0;
uint32_t previous1;
uint32_t temp1;
bool Sensor1Flag = 0;


unsigned long previousMicros = 0;
unsigned long highInterval = 0;
unsigned long lowInterval = 0;
bool triggered = 0;

static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};


const uint32_t syncMin = 61;    // Shortest high level for a sync pules
const uint32_t syncMax = 136;   // Longest high level for a sync pulse

uint32_t timerValueLow;
uint32_t timerValueHigh;

void IRAM_ATTR IntZero()
{
  if (digitalRead(SensorPin[0]) == HIGH) // we are inside a pulse
  {
    previousMicros = micros();
    flag = 1;
  }
  else    // falling edge pulse finished
  {
    highInterval = micros() - previousMicros;
    triggered = 1;
  }
}

void IRAM_ATTR IntOne()
{
  Sensor1Flag = 1;
}


void setup()
{
  Serial.begin(1000000);
  Serial.println("Hello");

  for (int i = 0; i < NumOfSensors; i++)
  {
    pinMode(SensorPin[i], INPUT);
  }

  if ( mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, SensorPin[0]) != ESP_OK)
  {
    Serial.println("Fail to init");
  }
  else Serial.println("Init Success!");

  // mcpwm_num, cap_sig, Edge 1 Positive 0 negative,
  if (mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0) != ESP_OK)
  {
    Serial.println("Fail to enable");
  }
  else Serial.println("Enable Success!");

  attachInterrupt(SensorPin[0], IntZero, CHANGE);
  attachInterrupt(SensorPin[1], IntOne, RISING);

  delay(2000);
}

void loop()
{
  int sensor = 1;

  if (flag)
  {
    flag = 0;
    previous = temp;
    temp = (mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0));
  }

  if (Sensor1Flag)
  {
    Sensor1Flag = 0;
    previous1 = temp1;
    temp1 = (mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP1));

    if (waiting4Pulse)
    {
      sensor = 1;
      angle[sensor][syncAxis] = ((((temp - previous) * 0.0125) - 4000.0) * PI) / 8333.0;
      Serial.print("Sensor Flag      ");
      Serial.print(sensor);
      Serial.print(": ");
      if (syncAxis == XAXIS) Serial.print("X ");
      else Serial.print("Y ");
      Serial.println(angle[sensor][syncAxis], 4);
      waiting4Pulse--; // We have received one of the 
    }

  }



  if (triggered)
  {
    triggered = 0;

    // SYNC PULSE
    if ( ((uint32_t)highInterval > syncMin) && ((uint32_t)highInterval <= syncMax) )//61 -> 136
    {
      answer = ((highInterval - 62) / 10.4);

      syncAxis = (int)(answer + 0.5) & 1;
      syncData = ((int)(answer + 0.5) & 2) >> 1;
      syncSkip = ((int)(answer + 0.5) & 4) >> 2;
      if (!syncSkip) waiting4Pulse = NumOfSensors;
      else waiting4Pulse = false;
    }

    // TIME PULSE
    else if ((uint32_t)highInterval < syncMin) // Valid time
    {
      sensor = 0;
      angle[sensor][syncAxis] = ((((temp1 - previous1) * 0.0125) - 4000.0) * PI) / 8333.0;
      Serial.print("Sensor triggered ");
      Serial.print(sensor);
      Serial.print(": ");
      if (syncAxis == XAXIS) Serial.print("X ");
      else Serial.print("Y ");
      Serial.println(angle[sensor][syncAxis], 4);
      waiting4Pulse--; // We have received one of the 
    }

    // NOT A VALID PULSE
    else waiting4Pulse = false;
  }


}
