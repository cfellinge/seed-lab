/*
  Main SEED Lab Code
  Fall 2023
  Cody J Fellinge
  Matt J Hatch
*/

#include "Arduino.h"
#include "MotorControl.h"
#include "StatusLEDControl.h"
#include "PositionMath.h"

// TIMER2 INTERRUPT VARIABLES
int count;
int lastCount;
byte timerReloadValue = 0x9C;

// // motor encoder data pins
int motorEncoderLeftA = 2;
int motorEncoderRightA = 3;

// misc variables
int secondsSinceStartup = 0;

// wheelbase width in meters
double wheelbaseWidth = 0.33;

StatusLEDControl clockLED(13);
StatusLEDControl taskLED(11);

MotorControl motorController(0);
PositionMath position(wheelbaseWidth);

double velocityTarget = 1;

ISR(TIMER2_COMPA_vect)
{
  count++;
  OCR2A = timerReloadValue;
}

void setup()
{
  // configure serial
  Serial.begin(115200);
  Serial.println("SEED Lab Robot Initializing");


  // configure timer 2 interrupt
  cli();
  TCCR0B = 0;
  OCR2A = timerReloadValue;
  TCCR2A = 1 << WGM21;
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
  TIMSK2 = (1 << OCIE2A);
  sei();
  Serial.println("Timer2 Interrupt Configured");


  // attach motor encoder interrupts
  attachInterrupt(digitalPinToInterrupt(motorEncoderLeftA), leftPinInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorEncoderRightA), rightPinInterrupt, CHANGE);
  
  Serial.println("Motor Encoder Interrupts Configured");

  motorController.begin();


  Serial.println("Beginning main loop:");
  Serial.println("--------------------------------------------------------------------------");

  motorController.setMotorMode(2);
  
  // smotorController.setVelocities(2.6, 2.6);
  motorController.setPositions(1.6, 0);
}

void loop()
{
  if (lastCount != count)
  {

    // do four times a second
    if (count % 25 == 0)
    {
      taskLED.onLED();
      clockLED.toggleLED();
      taskLED.offLED();
    }

    if (count % 10 == 1)
    {
      taskLED.onLED();
      motorController.updateMotorValues(100);
      position.updatePosition(0.1, motorController.getLeftVelocity(), motorController.getRightVelocity());
      taskLED.offLED();
    }

    // do every second
    if (count % 100 == 0)
    {
      taskLED.onLED();
      // Serial.println("1 second has passed");
      // Serial.println("Left count: " + (String)motorController.getLeftCount() + ", Right count: " + (String)motorController.getRightCount());
      // Serial.println("Seconds passed: " + (String)secondsSinceStartup);
      // Serial.println("Left m/s: " + (String)(motorController.getLeftVelocity()) + ", Right m/s: " + (String)(motorController.getRightVelocity()));
      // Serial.println("Left Voltage: " + (String)((double)leftRPMSet/255.0*8.0) + ", Right Voltage: " + (String)((double)rightRPMSet/255.0*8.0));
      // Serial.println("x: " + (String)(position.getX()) + ", y: " + (String)(position.getY()) + ", phi: " + (String)(position.getPhi()));
      Serial.println("Left pos: " + (String)(motorController.getLeftPosition() / PI) + " pi, Right pos: " + (String)(motorController.getRightPosition() / PI) + " pi");

      secondsSinceStartup++;
      taskLED.offLED();
    }

    // do every 10 seconds
    if (count == 1000)
    {
      taskLED.onLED();
      count = 0;
      taskLED.offLED();
    }

    lastCount = count;
  }
}

void leftPinInterrupt()
{
  motorController.leftPinInterrupt();
}

void rightPinInterrupt()
{
  motorController.rightPinInterrupt();
}