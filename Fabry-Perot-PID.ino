/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/
#include "Adafruit_MCP4725.h"
#include <PID_v1.h>

Adafruit_MCP4725 DAC;

#define PIN_INPUT 0
#define PIN_OUTPUT 3

int Read, pos = 0;
double maximum = 0.0;

double peaksearch()
{
    for (int i = 0; i < 4096; i++)
    {
        DAC.setVoltage(i, false);
        delay(100);
        Read = analogRead(A0);
        if (Read > maximum)
        {
            maximum = Read;
            pos = i;
        }
    }
  return (maximum*0.00115);
}

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  //initialize the variables we're linked to
  Input = analogRead(PIN_INPUT);
  Setpoint = peaksearch();
    Serial.begin(9600);
    DAC.begin(0x60);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  Input = analogRead(PIN_INPUT);
  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
}


