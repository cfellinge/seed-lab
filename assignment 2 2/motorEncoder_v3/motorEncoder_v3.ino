#include "MotorControl.h"
#include "StatusLEDControl.h"

// TIMER2 INTERRUPT VARIABLES
int count;
int lastCount;
byte timerReloadValue = 0x9C;

// MOTOR CONTROL PINS
// motor direction pins
int motorSwitchPin1 = 7;
int motorSwitchPin2 = 8;

// motor speed pins
int motorVoltagePin1 = 9;
int motorVoltagePin2 = 10;

// motor encoder data pins
int motorEncoderLeftA = 2;
int motorEncoderLeftB = 6;

int motorEncoderRightA = 3;
int motorEncoderRightB = 5;

// motor turn off
int motorTogglePin = 4;

// MOTOR DATA VARIABLES
// encoder variables
int thisRightA = LOW;
int lastRightA = LOW;

int thisLeftA = LOW;
int lastLeftA = LOW;

// // counting variables
int leftCount = 0;
int rightCount = 0;

int leftLastCount = 0;
int rightLastCount = 0;

double leftRPM = 0;
double rightRPM = 0;
double leftMetersPerSecond = 0;
double rightMetersPerSecond = 0;

// speed control variables
int leftRPMTarget = 0;
int rightRPMTarget = 0;

int leftRPMSet = 0;
int rightRPMSet = 0;


static double encoderCountsPerRotation = 800.0;


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

  // pinMode(motorSwitchPin1, OUTPUT);
  // pinMode(motorSwitchPin2, OUTPUT);

  // pinMode(motorVoltagePin1, OUTPUT);
  // pinMode(motorVoltagePin2, OUTPUT);

  // pinMode(motorTogglePin, OUTPUT);

  motorControl.begin();

  // enable motors
  digitalWrite(motorTogglePin, HIGH);

  digitalWrite(motorSwitchPin1, HIGH);
  digitalWrite(motorSwitchPin2, HIGH);

  leftRPMTarget = 10;
  rightRPMTarget = 10;
}

void loop()
{
  if (lastCount != count)
  {

    // do every half second
    if (count % 25 == 0)
    {
      statusLED.toggleLED();

      // check motor speeds
      if (abs(leftRPM) < (leftRPMTarget - 1)) {
        leftRPMSet += 2;
      }
      if (abs(leftRPM) > (leftRPMTarget + 1)) {
        leftRPMSet -= 2;
      }

      if (abs(rightRPM) < (rightRPMTarget - 1)) {
        rightRPMSet += 2;
      }
      if (abs(rightRPM) > (rightRPMTarget + 1)) {
        rightRPMSet -= 2;
      }

      analogWrite(motorVoltagePin1, leftRPMSet);
      analogWrite(motorVoltagePin2, rightRPMSet);


    }

    // debugging statements for every second
    if (count % 100 == 0)
    {
      // Serial.println("1 second has passed");
      // Serial.println("Left count: " + (String)leftCount + ", Right count: " + (String)rightCount);
      Serial.println("Seconds passed: " + (String)secondsSinceStartup);
      Serial.println("Left RPM: " + (String)leftRPM + ", Right RPM: " + (String)rightRPM);
      Serial.println("Left m/s: " + (String)leftMetersPerSecond + ", Right m/s: " + (String)rightMetersPerSecond);
      Serial.println("Left Voltage: " + (String)((double)leftRPMSet/255.0*8.0) + ", Right Voltage: " + (String)((double)rightRPMSet/255.0*8.0));
    }

    // controls for every second
    if (count % 100 == 0)
    {
      leftRPM = calculateRPM(leftCount, leftLastCount, 1000);
      leftMetersPerSecond = calculateMetersPerSecond(leftCount, leftLastCount, 1000);
      leftLastCount = leftCount;

      rightRPM = calculateRPM(rightCount, rightLastCount, 1000);
      rightMetersPerSecond = calculateMetersPerSecond(rightCount, rightLastCount, 1000);
      rightLastCount = rightCount;

      secondsSinceStartup++;
    }

    // do every 10 seconds
    if (count == 1000)
    {
      count = 0;

      
      // Serial.println("10 seconds have passed");
      leftRPMTarget = random(0, 100);
      rightRPMTarget = leftRPMTarget;
      Serial.println("RPM Targets set: " + (String)leftRPMTarget);
    }

    lastCount = count;
  }
}

double calculateRPM(int countsRotated, int lastCountsRotated, int numMilliSeconds)
{
  double numRotations = (double)(countsRotated - lastCountsRotated) / encoderCountsPerRotation;
  return numRotations * (numMilliSeconds / 1000.0) * 60.0;  //RETURN THIS FOR RPM VALUE
}

double calculateMetersPerSecond(int countsRotated, int lastCountsRotated, int numMilliSeconds)
{
  double numRotations = (double)(countsRotated - lastCountsRotated) / encoderCountsPerRotation;
  numRotations = numRotations * (numMilliSeconds / 1000.0) * 60.0;  //RETURN THIS FOR RPM VALUE
  return numRotations * 0.00764; //THIS IS M/S USING A WHEEL DIAMETER OF 14.6 CM, CAN BE CHANGED
}

void leftPinInterrupt()
{
  motorControl.leftPinInterrupt();
}

void rightPinInterrupt()
{
  motorControl.rightPinInterrupt();
}

// void leftClockwise()
// {
//   leftCount += 1;
// }

// void leftCounterClockwise()
// {
//   leftCount -= 1;
// }
// void rightClockwise()
// {
//   rightCount += 1;
// }

// void rightCounterClockwise()
// {
//   rightCount -= 1;
// }