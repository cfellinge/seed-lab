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

void setUpTimer1Interrupt() {
  cli();
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 1000;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

void setup() {
  // put your setup code here, to run once:
  pinMode(motorSwitchPin1, OUTPUT);
  pinMode(motorSwitchPin2, OUTPUT);
  
  pinMode(motorVoltagePin1, OUTPUT);
  pinMode(motorVoltagePin2, OUTPUT);

  pinMode(motorTogglePin, OUTPUT);

  // set up the interrupt for timer1 (see below)
  setUpTimer1Interrupt();
  
  // turn motors on
  digitalWrite(motorTogglePin, HIGH);

  digitalWrite(motorSwitchPin1, HIGH);
  digitalWrite(motorSwitchPin2, LOW);

  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(motorEncoderLeftA), leftPinInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorEncoderRightA), rightPinInterrupt, CHANGE);
}

int state = 1;

void loop() {
  // fade in from min to max in increments of 5 points:
  for (int fadeValue = 0; fadeValue <= 255; fadeValue += 2) {
    // sets the value (range from 0 to 255):
    analogWrite(motorVoltagePin1, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
    Serial.println("Left speed: " + (String)leftRPM + " RPM, right speed: " + (String)rightRPM + " RPM");
  }

  // fade out from max to min in increments of 5 points:
  for (int fadeValue = 255; fadeValue >= 0; fadeValue -= 2) {
    // sets the value (range from 0 to 255):
    analogWrite(motorVoltagePin1, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
    Serial.println("Left speed: " + (String)leftRPM + " RPM, right speed: " + (String)rightRPM + " RPM");
  }

  if (state == 0) {
    digitalWrite(motorSwitchPin1, HIGH);
    digitalWrite(motorSwitchPin2, LOW);
    state = 1;
  } 
  else {
    digitalWrite(motorSwitchPin1, LOW);
    digitalWrite(motorSwitchPin2, HIGH);
    state = 0;
  }
  
}


void leftPinInterrupt() {
  thisLeftA = digitalRead(motorEncoderLeftA);

  if (thisLeftA == HIGH && lastLeftA == LOW) {
    // logic for CW and CCW rotations
    if (digitalRead(motorEncoderLeftB) == HIGH) {
      leftClockwise();
    }
    else {
      leftCounterClockwise();
    }
  }

  // save current state of A
  lastLeftA = thisLeftA;
}

void rightPinInterrupt() {
  thisRightA = digitalRead(motorEncoderRightA);

  if (thisRightA == HIGH && lastRightA == LOW) {
    // logic for CW and CCW rotations
    if (digitalRead(motorEncoderRightB) == HIGH) {
      rightClockwise();
    }
    else {
      rightCounterClockwise();
    }
  }

  // save current state of A
  lastRightA = thisRightA;
}


void leftClockwise() {
  leftCount += 4;
  // Serial.print("Left CW ");
  // Serial.println(leftCount);
}

void leftCounterClockwise() {
  leftCount -= 4;
  // Serial.print("Left CCW ");
  // Serial.println(leftCount);
}
void rightClockwise() {
  rightCount += 4;
  // Serial.print("Right CW ");
  // Serial.println(rightCount);
}

void rightCounterClockwise() {
  rightCount -= 4;
  // Serial.print("Right CCW ");
  // Serial.println(rightCount);
}

ISR(TIMER1_COMPA_vect){
  leftRPM = ((double)(leftCount - leftLastCount));
  rightRPM = ((double)(rightCount - rightLastCount));

  leftLastCount = leftCount;
  rightLastCount = rightCount;

}