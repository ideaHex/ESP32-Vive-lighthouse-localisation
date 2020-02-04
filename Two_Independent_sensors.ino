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
float answer[NumOfSensors];
int32_t X[NumOfSensors];
int32_t Y[NumOfSensors];
float yAngle[NumOfSensors];
float xAngle[NumOfSensors];
uint32_t interval[NumOfSensors];
float angle[NumOfSensors][2];

int syncAxis[NumOfSensors];
int syncData[NumOfSensors];
int syncSkip[NumOfSensors];


int waiting4Pulse[NumOfSensors];

uint32_t previous[NumOfSensors];
uint32_t temp[NumOfSensors];
bool flag[NumOfSensors];
uint32_t previous1;
uint32_t temp1;
bool Sensor1Flag = 0;

int sensor = 0;


unsigned long previousMicros[NumOfSensors];
unsigned long highInterval[NumOfSensors];
unsigned long lowInterval[NumOfSensors];
bool triggered[NumOfSensors];

static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};

static mcpwm_capture_signal_t CAP[2] = {MCPWM_SELECT_CAP0, MCPWM_SELECT_CAP1};

const uint32_t syncMin = 61;    // Shortest high level for a sync pules
const uint32_t syncMax = 136;   // Longest high level for a sync pulse

uint32_t timerValueLow;
uint32_t timerValueHigh;



void ISR(int sensor)
{
  if (digitalRead(SensorPin[sensor]) == HIGH) // we are inside a pulse
  {
    previousMicros[sensor] = micros();
    flag[sensor] = 1;
  }
  else    // falling edge pulse finished
  {
    highInterval[sensor] = micros() - previousMicros[sensor];
    triggered[sensor] = 1;
  }
}

void IRAM_ATTR IntZero()
{
  ISR(0);

}

void IRAM_ATTR IntOne()
{
  ISR(1);
}


void setup()
{
  Serial.begin(1000000);

  for (int i = 0; i < NumOfSensors; i++)
  {
    pinMode(SensorPin[i], INPUT);
  }

  if ( mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, SensorPin[0]) != ESP_OK)
  {
    Serial.println("Fail to init");
  }

  if ( mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, SensorPin[1]) != ESP_OK)
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
  if (mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, MCPWM_POS_EDGE, 0) != ESP_OK)
  {
    Serial.println("Fail to enable");
  }
  else Serial.println("Enable Success!");


  attachInterrupt(SensorPin[0], IntZero, CHANGE);
  attachInterrupt(SensorPin[1], IntOne, CHANGE);

  delay(2000);
}

void loop()
{
  for (int i = 0; i < NumOfSensors; i++)
  {
    if (flag[i])
    {
      flag[i] = false;
      previous[i] = temp[i];
      //temp[i] = (mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0));//CAP[sensor]));
      temp[i] = (mcpwm_capture_signal_get_value(MCPWM_UNIT_0, CAP[i]));
    }

    if (triggered[i])
    {
      triggered[i] = 0;

      // SYNC PULSE
      if ( ((uint32_t)highInterval[i] > syncMin) && ((uint32_t)highInterval[i] <= syncMax) )//61 -> 136
      {
        // TO DO this needs work as syncaxis seems to get mixed up
        answer[i] = ((highInterval[i] - 62) / 10.4);
        syncAxis[i] = (int)(answer[i] + 0.5) & 1; // +0.5 to round up
        syncData[i] = ((int)(answer[i] + 0.5) & 2) >> 1;
        syncSkip[i] = ((int)(answer[i] + 0.5) & 4) >> 2;
      }

      // TIME PULSE
      else if ((uint32_t)highInterval[i] < syncMin) // Valid time
      {
        float tempTime = ((temp[i] - previous[i]) * 0.0125) - 4000;
        //if(tempTime < 4000) tempTime = 4000;
        //tempTime = tempTime - 4000;

        //angle[sensor][syncAxis[sensor]] = ((((temp[sensor] - previous[sensor]) * 0.0125) - 4000.0) * PI) / 8333.0;
        if (tempTime > -3500 && tempTime < 3500) // Check if valid data
        {
          angle[i][syncAxis[i]] = tempTime;
        }

      }
    }
  }
  //int j = 0;
  for (int j = 0; j < NumOfSensors; j++)
  {
    Serial.print(angle[j][XAXIS]);
    Serial.print(" ");
    Serial.print(angle[j][YAXIS]);
    Serial.print(" ");
  }
  //Serial.print(" ");
  Serial.print(-4000);
  Serial.print(" ");
  Serial.print(4000);
  Serial.println();
}
