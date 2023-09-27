#include "MotorControl.h"
#include "StatusLEDControl.h"
#include "PositionMath.h"
#include "PiCommunication.h"

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
StatusLEDControl taskLED(12);

MotorControl motorControl();
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

  

  // configure timer 2 interrupt
  cli();
  TCCR0B = 0;
  OCR2A = timerReloadValue;
  TCCR2A = 1 << WGM21;
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
  TIMSK2 = (1 << OCIE2A);
  sei();

  // attach motor encoder interrupts
  attachInterrupt(digitalPinToInterrupt(motorEncoderLeftA), leftPinInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorEncoderRightA), rightPinInterrupt, CHANGE);

  // set output pins to output
  pinMode(LED_BUILTIN, OUTPUT);
  PiCommunication.begin();
  motorControl.begin();

  motorControl.setVelocities(2.6, 2.6);
}

void loop()
{
  if (lastCount != count)
  {

    // do four times a second
    if (count % 25 == 0)
    {
      clockLED.toggleLED();
      motorControl.updateMotorValues(250);
    }

    if (count % 25 == 1)
    {
      position.updatePosition(0.25, motorControl.getLeftVelocity(), motorControl.getRightVelocity());
      Serial.println("x: " + (String)(position.getX()) + ", y: " + (String)(position.getY()) + ", phi: " + (String)(position.getPhi()));
    }

    if (count % 25 == 2)
    {
      PiCommunication.receive;
      int quadrant = PiCommunication.getInstruction;
    }

    // do every second
    if (count % 100 == 0)
    {
      // Serial.println("1 second has passed");
      // Serial.println("Left count: " + (String)motorControl.getLeftCount() + ", Right count: " + (String)motorControl.getRightCount());
      // Serial.println("Seconds passed: " + (String)secondsSinceStartup);
      // Serial.println("Left m/s: " + (String)(motorControl.getLeftVelocity()) + ", Right m/s: " + (String)(motorControl.getRightVelocity()));
      // Serial.println("Left Voltage: " + (String)((double)leftRPMSet/255.0*8.0) + ", Right Voltage: " + (String)((double)rightRPMSet/255.0*8.0));

      secondsSinceStartup++;
    }

    // do every 10 seconds
    if (count == 1000)
    {
      count = 0;
    }

    lastCount = count;
  }
}

void leftPinInterrupt()
{
  motorControl.leftPinInterrupt();
}

void rightPinInterrupt()
{
  motorControl.rightPinInterrupt();
}