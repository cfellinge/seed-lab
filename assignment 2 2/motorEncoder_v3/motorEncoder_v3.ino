#include "MotorControl.h"
#include "StatusLEDControl.h"

// TIMER2 INTERRUPT VARIABLES
int count;
int lastCount;
byte timerReloadValue = 0x9C;

// // motor encoder data pins
int motorEncoderLeftA = 2;
int motorEncoderRightA = 3;

// misc variables
int secondsSinceStartup = 0;

StatusLEDControl statusLED(13);
MotorControl motorControl(1);

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

  motorControl.begin();

   motorControl.setVelocities(1, 1);
}

void loop()
{
  if (lastCount != count)
  {

    // do four times a second
    if (count % 25 == 0)
    {
      statusLED.toggleLED();
      motorControl.updateMotorValues(250);
    }

    // do every second
    if (count % 100 == 0)
    {
      // Serial.println("1 second has passed");
      Serial.println("Left count: " + (String)motorControl.getLeftCount() + ", Right count: " + (String)motorControl.getRightCount());
      // Serial.println("Seconds passed: " + (String)secondsSinceStartup);
      Serial.println("Left m/s: " + (String)(motorControl.getLeftVelocity() * 10000) + ", Right m/s: " + (String)(motorControl.getRightVelocity() * 10000));
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