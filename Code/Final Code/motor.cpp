#include "motor.h"
#include <Arduino.h> 

void Forward(int Forwardpin, int Backwardpin, int speed)
{
analogWrite(Forwardpin, speed);
digitalWrite(Backwardpin, LOW);
}
void Backward(int Forwardpin, int Backwardpin, int speed)
{
  analogWrite(Backwardpin, speed);
  digitalWrite(Forwardpin, LOW);
}
void Stop(int Forwardpin, int Backwardpin)
{
  digitalWrite(Forwardpin, LOW);
  digitalWrite(Backwardpin, LOW);
}