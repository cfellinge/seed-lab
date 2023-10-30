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

// motor encoder data pins
int motorEncoderLeftA = 2;
int motorEncoderRightA = 3;

// misc variables
long millisecondsSinceStartup = 0;
int secondsSinceStartup = 0;

// wheelbase width in meters
const double WHEELBASE_WIDTH = 0.37;

// wheel radius in meters
const double WHEEL_RADIUS = 0.0725;

// object definitions
StatusLEDControl clockLED(13);

StatusLEDControl taskLED(11);

MotorControl motorController(0);

PositionMath position(WHEELBASE_WIDTH);

Movement movement(motorController, position, WHEEL_RADIUS);

PiCommunication piCommunication;

// demo2 states
enum DEMO_2_STATE
{
  RESET_STATE,
  ACQUIRE_SIGNAL,
  SPIN_2_ZERO,
  STOP_1,
  GO_TO_COORDS,
  STOP_2,
  SPIN_90,
  CIRCLE_TIME,
  STOP_3,
  TEST_1_DONE,
  TEST_2_DONE
};

// demo2 temp variables
DEMO_2_STATE demo2State;
const int testMode = 2;

// 

// timer2 ISR
ISR(TIMER2_COMPA_vect)
{
  count++;
  OCR2A = timerReloadValue;
}

// runs once at robot startup
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

  // initialize motor controller and other objects
  motorController.begin();

  piCommunication.begin();
  Wire.onReceive(piCommsInterrupt);

  Serial.println("Pi Communication initialized");

  Serial.println("Beginning main loop:");
  Serial.println("--------------------------------------------------------------------------");
}

void loop()
{
  // 100 Hz max. frequency
  if (lastCount != count)
  {
    // taskLED makes sure we don't get stuck inside a function
    taskLED.onLED();

    // increment time variables and set lastCount = count
    millisecondsSinceStartup += 10;
    secondsSinceStartup = millisecondsSinceStartup / 1000;
    lastCount = count;

    // 50 Hz
    // update all objects
    if (count % 2 == 0)
    {
      motorController.updateMotorValues(20);
      movement.updateMovement(20);
      position.updatePosition(20, motorController.getLeftVelocity(), motorController.getRightVelocity());
    }

    // 50 Hz
    // FSM
    if (count %2 == 1) {
      
    }

    // 4 Hz
    // Blink clock LED
    if (count % 25 == 1)
    {
      // LED Blink
      clockLED.toggleLED();
    }

    // 2 Hz
    // print debug statements
    if (count % 50 == 1)
    {
      printDebugStatements();
    }

    // rest count every 10 seconds to avoid overflow
    if (count % 10000 == 1)
    {
      count = 0;
    }

    taskLED.offLED();
  }
}

void fsmUpdate() {
  switch(demo2State) {
    case RESET_STATE:
      demo2State = ACQUIRE_SIGNAL;
      break;

    case ACQUIRE_SIGNAL:
      if ()


      break;

    case SPIN_2_ZERO:

      break;
    
    case STOP_1:

      break;    
    
    case GO_TO_COORDS:

      break;

    case STOP_2:

      break;

    case SPIN_90:

      break;
    
    case CIRCLE_TIME:

      break;

    case STOP_3:

      break;

    case TEST_1_DONE:

      break;

    case TEST_2_DONE:

      break;
    
    default:
      Serial.println("ERROR: Default FSM State reached. Exiting.");
      exit(-1);
      break;

  }
}


void printDebugStatements()
{
  // TEST PRINTS
  // Serial.println("\n\nDEBUG INFO:\n");

  // COUNTS
  // Serial.println("Left count: " + (String)motorController.getLeftCount() + ", Right count: " + (String)motorController.getRightCount());

  // SECONDS PASSED
  // Serial.println("ms passed: " + (String)millisecondsSinceStartup);

  // M/S
  // Serial.println("Left m/s: " + (String)(motorController.getLeftVelocity()) + ", Right m/s: " + (String)(motorController.getRightVelocity()));

  // VOLTAGE
  // Serial.println("Left Voltage: " + (String)((double)motorController.getLeftWriteValue() / 255.0 * 8.0) + ", Right Voltage: " + (String)((double)motorController.getRightWriteValue() / 255.0 * 8.0));

  // X, Y, PHI
  // Serial.println("x: " + (String)(position.getX()) + ", y: " + (String)(position.getY()) + ", rho: " + (String)(position.getRho()) + ", phi: " + (String)(position.getPhi() / PI) + " pi");

  // WHEEL POSITIONS
  // Serial.println("Left pos: " + (String)(motorController.getLeftPosition() / PI) + " pi, Right pos: " + (String)(motorController.getRightPosition() / PI) + " pi");

  // VA, DV
  // Serial.println("VA: " + (String)movement.getVA() + ", DV: " + (String)movement.getDV());

  // RHO, PHI TARGETS
  // Serial.println("Rho goal: " + (String)movement.getRhoTarget() + ", Phi goal: " + (String)(movement.getPhiTarget() / PI) + " pi");

  // X, Y TARGETS
  // Serial.println("X goal: " + (String)movement.getXTarget() + ", Y goal: " + (String)(movement.getYTarget()));

  // FORWARD, ROTATIONAL VELOCITY
  // Serial.println("Forwards velocity: " + (String)movement.getForwardVel() + ", Rotational velocity: " + (String)movement.getRotationalVel());
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