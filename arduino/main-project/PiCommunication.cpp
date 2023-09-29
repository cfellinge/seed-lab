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
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;
volatile uint8_t reply = 0;

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
void PiCommunication::printReceived()
{
  // Print on serial console
  Serial.print("Offset received: ");
  Serial.println(offset);
  Serial.print("Message Length: ");
  Serial.println(msgLength);
  Serial.print("Instruction received: ");
  for (int i = 0; i < msgLength; i++)
  {
    Serial.print(String(instruction[i]) + "\t");
  }
  Serial.println("");
}

// CURRENTLY ASSUMES ONLY ONE INT OF DATA
int PiCommunication::getInstruction()
{
  // for (int i = 0; i < sizeof(instruction); i++) {
  //   Serial.print(instruction[i]);
  // }
  // Serial.print("\n");
  return instruction[msgLength - 1];
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

// void PiCommunication::request()
// {
//   // According to the Wire source code, we must call write() within the requesting ISR
//   // and nowhere else. Otherwise, the timing does not work out. See line 238:
//   // https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/Wire/src/Wire.cpp
//   uint8_t localReply = reply;
//   localReply += 100;
//   Serial.print(localReply);
//   Wire.write(localReply);
//   reply = 0;
// }
