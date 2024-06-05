#include <Servo.h>
#ifndef DRIVING_H
#define DRIVING_H
void Drive(Servo myServo,int Foward, int Backward, int speed,int leftIr, int rightIr, int trigpin, int echopin);
void TurningLeft(Servo myServo,int Forward, int Backward, int speed);
void TurningRight(Servo myServo,int Forward, int Backward, int speed);
void Straight(Servo myServo,int Forward, int Backward, int speed);
void Back(Servo myServo,int Forward, int Backward, int speed);
#endif