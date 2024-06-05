#include "CalcSensor.h"
#include <Arduino.h> 
int LightSensorValue(int IrValue)
{
    int num = digitalRead(IrValue); // 0 mean space to turn 1 mean wall
    return  num;
}

long SoundDistance(int trigPin, int echoPin)
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
 
    // Read the signal from the sensor: a HIGH pulse whose
    // duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    pinMode(echoPin, INPUT);
    long duration = pulseIn(echoPin, HIGH);
 
    // Convert the time into a distance
    long cm = (duration/2) * 0.0343; // Divide by 29.1 or multiply by 0.0343
    return cm;
}