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

void PiCommunication::updatePi(int numMillis) {
  // If there is data on the buffer, read it
  if (msgLength > 0) {
    // Serial.print("PRINT RECEIVED:");
    printReceived();
    msgLength = 0;
  }
}

// printReceived helps us see what data we are getting from the leader
// interrupt
void PiCommunication::printReceived()
{
  // Serial.print("SendArray: "); 
  for (int i=0;i<msgLength;i++) {
    // Serial.print(instruction[i]);
    // Serial.print(" ");
   }
  Serial.print("\n"); 
  // Serial.print("Angle received: ");
  for (int i=0;i<6;i++) {
    angle[i] = uint8_t(instruction[i]) - 48;
    // Serial.print(angle[i]);
    // Serial.print(" ");
   }
  if(angle[0] == 253){
    finalAngle = -((float)angle[1] + (float)angle[3]/10.0 + (float)angle[4]/100.0 + (float)angle[5]/1000.0);
  }
  else{
   finalAngle = (float)angle[0] + (float)angle[2]/10.0 + (float)angle[3]/100.0 + (float)angle[4]/1000.0 + (float)angle[5]/10000.0;
  }
  //  Serial.println("\n");
  //  Serial.println("Angle Float: ");
  //  Serial.print(finalAngle);
  // Serial.println("\n");
  // Serial.print("Distance received: ");
  for (int i=0;i<6;i++) {
    dist[i] = uint8_t(instruction[i+6]) - 48;
    // Serial.print(dist[i]);
   }
    if(dist[0] == 253){
    finalDist = -((float)dist[1] + (float)dist[3]/10.0 + (float)dist[4]/100.0 + (float)dist[5]/1000.0);
  }
  else{
   finalDist = (float)dist[0] + (float)dist[2]/10.0 + (float)dist[3]/100.0 + (float)dist[4]/1000.0 + (float)dist[5]/10000.0;
  }
   
  // Serial.println("\n");
  // Serial.println("Distance Float: ");
  // Serial.print(finalDist);
  // Serial.println("\n");
  
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
