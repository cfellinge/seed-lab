/*
    PiCommunication.cpp
    Code to control motors on SEED Lab robot
*/

int pin;

#include "Arduino.h"
#include "PiCommunication.h"
#include <Wire.h>

// Global Varialbes for I2C communication, taken from Exercise 1B
volatile uint8_t offset = 0;
volatile uint8_t instruction[12] = {0};
volatile uint8_t angle[6] = {0};
volatile uint8_t dist[6] = {0};
volatile uint8_t msgLength = 0;
volatile float finalAngle = NAN;
volatile float finalDist = NAN;

int MY_ADDR = 8;

PiCommunication::PiCommunication()
{
}

void PiCommunication::begin()
{
  Wire.begin(MY_ADDR);
  // Wire.onReceive(piCommsInterrupt);
  // Wire.onRequest(request);
}

// printReceived helps us see what data we are getting from the leader
// interrupt
void PiCommunication::printReceived()
{
  Serial.print("Offset received: ");
  Serial.println(offset);
  Serial.print("Message Length: ");
  Serial.println(msgLength);
  Serial.print("Angle received: ");
  for (int i = 0; i < 6; i++)
  {
    angle[i] = uint8_t(instruction[i]) - 48;
    Serial.print(angle[i]);
  }
  if (angle[0] = 253)
  {
    for (int i = 0; i < 5; i++)
    {
      angle[i] = angle[i + 1];
    }
  }
  finalAngle = (float)angle[5] / 1000000.0 + (float)angle[4] / 100000.0 + (float)angle[3] / 10000.0 + (float)angle[2] / 1000.0 + (float)angle[1] / 100.0 + angle[0] / 10.0;
  Serial.println("\n");
  Serial.println("Angle Float: ");
  Serial.print(finalAngle);
  Serial.println("\n");
  Serial.print("Distance received: ");
  for (int i = 0; i < 6; i++)
  {
    dist[i] = uint8_t(instruction[i + 6]) - 48;
    Serial.print(dist[i]);
  }
  finalDist = (float)dist[5] / 1000000.0 + (float)dist[4] / 100000.0 + (float)dist[3] / 10000.0 + (float)dist[2] / 1000.0 + (float)dist[1] / 100.0 + dist[0] / 10.0;
  Serial.println("\n");
  Serial.println("Distance Float: ");
  Serial.print(finalDist);
  Serial.println("\n");
  
  if (abs(finalAngle) > 2*PI) {
    finalAngle = NAN;
  }

  if (abs(finalDist) > 15) {
    finalDist = NAN;
  }
}


// function called when an I2C interrupt event happens
void PiCommunication::receive(int)
{
  // Set the offset, this will always be the first byte.
  offset = Wire.read();
  // If there is information after the offset, it is telling us more about the command.
  while (Wire.available())
  {
    instruction[msgLength] = Wire.read();
    msgLength++;
  }
}

float PiCommunication::getAngle() {
  return finalAngle;
}

float PiCommunication::getDistance() {
  return finalDist;
}