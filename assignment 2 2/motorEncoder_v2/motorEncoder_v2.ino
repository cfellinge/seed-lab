// TIMER2 INTERRUPT VARIABLES
int count;
int lastCount;
byte timerReloadValue = 0x9C;
static boolean output = HIGH;

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

// counting variables
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


// counting variables
double xPos = 0;
double yPos = 0;
double phi = 0;
double wheelBaseWidth = 0.33;

static double encoderCountsPerRotation = 800.0;

// STATE VARIABLES
boolean clockLEDState = LOW;


// misc variables
int secondsSinceStartup = 0;

int startTime = 0;

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

  pinMode(motorSwitchPin1, OUTPUT);
  pinMode(motorSwitchPin2, OUTPUT);

  pinMode(motorVoltagePin1, OUTPUT);
  pinMode(motorVoltagePin2, OUTPUT);

  pinMode(motorTogglePin, OUTPUT);

  // enable motors
  digitalWrite(motorTogglePin, HIGH);

  digitalWrite(motorSwitchPin1, HIGH);
  digitalWrite(motorSwitchPin2, HIGH);

  startTime = millis();


}

int printToConsole = 1;

void loop()
{
  if (lastCount != count)
  {

    // do every half second
    if (count % 25 == 0)
    {
      flash();

      // // check motor speeds
      // if (abs(leftRPM) < (leftRPMTarget - 1)) {
      //   leftRPMSet += 2;
      // }
      // if (abs(leftRPM) > (leftRPMTarget + 1)) {
      //   leftRPMSet -= 2;
      // }

      // if (abs(rightRPM) < (rightRPMTarget - 1)) {
      //   rightRPMSet += 2;
      // }
      // if (abs(rightRPM) > (rightRPMTarget + 1)) {
      //   rightRPMSet -= 2;
      // }

      // // analogWrite(motorVoltagePin1, leftRPMSet);
      // // analogWrite(motorVoltagePin2, rightRPMSet);


    }

    // debugging statements for every second
    if (count % 100 == 0)
    {
      // Serial.println("1 second has passed");
      // Serial.println("Left count: " + (String)leftCount + ", Right count: " + (String)rightCount);
      // Serial.println("Seconds passed: " + (String)secondsSinceStartup);
      // Serial.println("Left RPM: " + (String)leftRPM + ", Right RPM: " + (String)rightRPM);
      // Serial.println("Left m/s: " + (String)leftMetersPerSecond + ", Right m/s: " + (String)rightMetersPerSecond);
      // Serial.println("Left Voltage: " + (String)((double)leftRPMSet/255.0*8.0) + ", Right Voltage: " + (String)((double)rightRPMSet/255.0*8.0));
     
      // 2b print
      Serial.println((String)xPos + "\t" + (String)yPos + "\t" + (String)phi);
      
    }

    // controls for 10x every second
    if (count % 10 == 0)
    {
      leftRPM = calculateRPM(leftCount, leftLastCount, 100);
      leftMetersPerSecond = calculateMetersPerSecond(leftCount, leftLastCount, 100);
      leftLastCount = leftCount;

      rightRPM = calculateRPM(rightCount, rightLastCount, 100);
      rightMetersPerSecond = calculateMetersPerSecond(rightCount, rightLastCount, 100);
      rightLastCount = rightCount;

      xPos = calculateX(xPos, 0.1, phi, leftMetersPerSecond, rightMetersPerSecond);
      yPos = calculateY(yPos, 0.1, phi, leftMetersPerSecond, rightMetersPerSecond);
      phi = calculatePhi(phi, 0.1, wheelBaseWidth, leftMetersPerSecond, rightMetersPerSecond);

      // 2a print
      // if (secondsSinceStartup < 3) Serial.println((String)secondsSinceStartup + "\t" + (String)(200/255.0*8.0) + "\t" + (String)leftMetersPerSecond);

      
      // debugging values
      // Serial.print((String)xPos + "\t" + (String)yPos + "\t" + (String)phi + "\t");
      // Serial.println((String)secondsSinceStartup + "\t" + (String)((double)leftRPMSet/255.0*8.0) + "\t" + (String)leftMetersPerSecond + "\t" + (String)rightMetersPerSecond);
      
    }

    // // STEP UNIT TEST SHIT
    // if (count == 100) {
    //   Serial.println("STEP ON");
    //   analogWrite(motorVoltagePin1, 200);
    // }

    // if (count == 300) {
    //   Serial.println("STEP OFF");
    //   analogWrite(motorVoltagePin1, 0);
    //   printToConsole = 0;
    // }

    if (count % 100 == 0) {
      secondsSinceStartup++;
    }

    // do every 10 seconds
    if (count == 1000)
    {
      count = 0;

      // Serial.println("xPos \tyPos \tphi \tsec. \tvolts \t l m/s \t r m/s");
      
      // Serial.println("10 seconds have passed");
      // leftRPMTarget = random(0, 100);
      // rightRPMTarget = leftRPMTarget;
      // Serial.println("RPM Targets set: " + (String)leftRPMTarget);
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
  // numRotations = numRotations * 0.007644;
  return numRotations; 
}

double calculateX(double xOld, double deltaT, double phiOld, double velocityLeft, double velocityRight) {
  return xOld + deltaT * cos(phiOld) * (velocityLeft + velocityRight) / 2;
}

double calculateY(double yOld, double deltaT, double phiOld, double velocityLeft, double velocityRight) {
  return yOld + deltaT * sin(phiOld) * (velocityLeft + velocityRight) / 2;
}

double calculatePhi(double phiOld, double deltaT, double wheelBaseWidth, double velocityLeft, double velocityRight) {
  double phiTemp = phiOld + deltaT * (velocityLeft - velocityRight) / wheelBaseWidth;
  if (phiTemp > 6.283) phiTemp = 0;
  if (phiTemp < 0) phiTemp = 6.283;
  return phiTemp;
}

void flash()
{
  digitalWrite(LED_BUILTIN, clockLEDState);
  clockLEDState = !clockLEDState;
}

void leftPinInterrupt()
{
  thisLeftA = digitalRead(motorEncoderLeftA);

  if (thisLeftA == HIGH && lastLeftA == LOW)
  {
    // logic for CW and CCW rotations
    if (digitalRead(motorEncoderLeftB) == HIGH)
    {
      leftCounterClockwise();
    }
    else
    {
      leftClockwise();
    }
  }

  // save current state of A
  lastLeftA = thisLeftA;
}

void rightPinInterrupt()
{
  thisRightA = digitalRead(motorEncoderRightA);

  if (thisRightA == HIGH && lastRightA == LOW)
  {
    // logic for CW and CCW rotations
    if (digitalRead(motorEncoderRightB) == HIGH)
    {
      rightCounterClockwise();
    }
    else
    {
      rightClockwise();
    }
  }

  // save current state of A
  lastRightA = thisRightA;
}

void leftClockwise()
{
  leftCount -= 1;
}

void leftCounterClockwise()
{
  leftCount += 1;
}
void rightClockwise()
{
  rightCount += 1;
}

void rightCounterClockwise()
{
  rightCount -= 1;
}