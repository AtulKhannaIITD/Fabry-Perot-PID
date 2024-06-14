#include "Adafruit_MCP4725.h"

Adafruit_MCP4725 DAC;

const int INPUT_PIN = A0;
//const int OUTPUT_PIN = DD3;

double dt, last_time;
double integral, previous, output = 0;
double kp, ki, kd;
double setpoint = 900.00;

void setup()
{
  kp = 3;
  ki = 0.00;
  kd = 0.0;
  last_time = 0;
  Serial.begin(9600);
  DAC.begin(0x60);
  DAC.setVoltage(0, false);
  for(int i = 0; i < 50; i++)
  {
    Serial.print(setpoint);
    Serial.print(",");
    Serial.println(0);
    delay(100);
  }
  delay(100);
}

void loop()
{
  double now = millis();
  dt = (now - last_time)/1000.00;
  last_time = now;

  double actual = map(analogRead(INPUT_PIN), 0, 1024, 0, 4096);
  double error = setpoint - actual;
  output = pid(error);

   DAC.setVoltage(output, false);

  // Setpoint VS Actual
  Serial.print(setpoint);
  Serial.print(",");
  Serial.println(actual);

  // Error
  //Serial.println(error);

  delay(300);
}

double pid(double error)
{
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  return output;
}