#include "motor.h"
#include "CalcSensor.h"
#include "Driving.h"
#include <Servo.h>
#include <Arduino.h>


void TurningLeft(Servo myServo,int Foward, int Backward, int speed) 
// Precondition 2 IRSensors going to be use(left and right) so if Right sensor is 0 and left is 1 
// Turn left so this methof will be called
{
    myServo.write(135);
     Forward(Foward, Backward, speed);

}
void TurningRight(Servo myServo,int Foward, int Backward, int speed)
// Precondition 2 IRSensors going to be use(left and right) so if Right sensor is 1v and left is 0 
// Turn right so this methof will be called
{
     myServo.write(45);
      Forward(Foward, Backward, speed);

}
void Straight(Servo myServo,int Foward, int Backward, int speed)
// If both sensors are 1 then the car will go straight so this method will be called
{
   
    Forward(Foward, Backward, speed);
}
void Back(Servo myServo,int Foward, int Backard, int speed)
// This method will be called by the floodfill method if we have to go back
// Also will be called before every turn
//While it's called encoder checks if 1 or 2 cm is reach and if it is it will stop
// and begin to turn
{
    Backward(Foward,Backard,speed);
}