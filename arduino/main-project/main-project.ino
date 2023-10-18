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
#include "PiCommunication.h"
#include "Movement.h"

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
double wheelbaseWidth = 0.37;

StatusLEDControl clockLED(13);
StatusLEDControl taskLED(11);

MotorControl motorController(0);
PositionMath position(wheelbaseWidth);
Movement movement(motorController, position);

PiCommunication piCommunication;

// demo1 temp variables
int demo1TrialMode = 0;
double targetMeters;
double targetRadians;
int stallCounter = 0;

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

  piCommunication.begin();
  Wire.onReceive(piCommsInterrupt);

  Serial.println("Pi Communication initialized");

  Serial.println("Beginning main loop:");
  Serial.println("--------------------------------------------------------------------------");

  demo1TrialMode = 2;

  if (demo1TrialMode == 1)
  {
    // TRIAL 1:
    double targetFeet = 7;

    targetMeters = targetFeet / 3.281;
    movement.moveForwards(targetMeters);
  }
  else if (demo1TrialMode == 2)
  {
    // TRIAL 2:
    double targetDegrees = 90;
    double targetFeet = 2;

    targetRadians = (targetDegrees * PI) / 180.0;
    movement.rotateLeft(targetRadians);
    targetMeters = targetFeet / 3.281;
  }

  // TRIAL 3:
}

void loop()
{
  if (lastCount != count)
  {
    // do four times a second
    if (count % 25 == 0)
    {
      // LED Blink
      taskLED.onLED();
      clockLED.toggleLED();
      taskLED.offLED();
    }

    if (count % 2 == 1)
    {
      // Update values read from and programmed to motor
      taskLED.onLED();

      motorController.updateMotorValues(20);

      movement.updateMovement(20);

      position.updatePosition(20, motorController.getLeftVelocity(), motorController.getRightVelocity());

      if (demo1TrialMode == 1)
      {
        // TRIAL 1:
        if (abs(position.getRho() - targetMeters) < 0.001)
        {
          stallCounter++;
        }
        if (stallCounter > 100)
        {
          movement.stop();
        }
      }
      else if (demo1TrialMode == 2)
      {
        // trial 3
        if (abs(position.getPhi() - targetRadians) < 0.01)
        {
          stallCounter++;
        }
        if (stallCounter > 100)
        {
          stallCounter = 0;
          demo1TrialMode = 1;
          movement.moveForwards(targetMeters);
        }
      }

      taskLED.offLED();
    }

    if (count % 25 == 2)
    {
    }

    // do every second
    if (count % 50 == 3)
    {
      taskLED.onLED();
      // TEST PRINTS
      Serial.println("1 second has passed");

      // COUNTS
      Serial.println("Left count: " + (String)motorController.getLeftCount() + ", Right count: " + (String)motorController.getRightCount());

      // SECONDS PASSED
      Serial.println("Seconds passed: " + (String)secondsSinceStartup);

      // M/S
      Serial.println("Left m/s: " + (String)(motorController.getLeftVelocity()) + ", Right m/s: " + (String)(motorController.getRightVelocity()));

      // VOLTAGE
      Serial.println("Left Voltage: " + (String)((double)motorController.getLeftWriteValue() / 255.0 * 8.0) + ", Right Voltage: " + (String)((double)motorController.getRightWriteValue() / 255.0 * 8.0));

      // X, Y, PHI
      Serial.println("x: " + (String)(position.getX()) + ", y: " + (String)(position.getY()) + ", rho: " + (String)(position.getRho()) + ", phi: " + (String)(position.getPhi() / PI) + " pi");

      // WHEEL POSITIONS
      // Serial.println("Left pos: " + (String)(motorController.getLeftPosition() / PI) + " pi, Right pos: " + (String)(motorController.getRightPosition() / PI) + " pi");

      // VA, DV
      Serial.println("VA: " + (String)movement.getVA() + ", DV: " + (String)movement.getDV());

      // RHO, PHI TARGETS
      Serial.println("Rho goal: " + (String)movement.getRhoTarget() + ", Phi goal: " + (String)(movement.getPhiTarget() / PI) + " pi");

      // X, Y TARGETS
      Serial.println("X goal: " + (String)movement.getXTarget() + ", Y goal: " + (String)(movement.getYTarget()));

      // FORWARD, ROTATIONAL VELOCITY
      Serial.println("Forwards velocity: " + (String)movement.getForwardVel() + ", Rotational velocity: " + (String)movement.getRotationalVel());

      Serial.println("\n");

      // if (secondsSinceStartup < 10) Serial.println((String)count + "\t" + (String)(movement.getForwardVel() * 1000) + "\t" + (String)(position.getRho() * 1000));

      taskLED.offLED();
    }

    if (count % 100 == 0)
    {
      secondsSinceStartup++;
    }

    // do every 5 seconds
    if (count == 5000)
    {
      taskLED.onLED();
      count = 0;

      // motorController.setPositions(positionTarget, positionTarget);
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

void piCommsInterrupt(int howMany)
{
  piCommunication.receive(howMany);
}