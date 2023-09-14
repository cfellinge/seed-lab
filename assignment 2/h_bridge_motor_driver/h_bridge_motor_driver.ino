// motor direction pins
int motorSwitchPin1 = 7;
int motorSwitchPin2 = 8;

// motor speed pins
int motorVoltagePin1 = 9;
int motorVoltagePin2 = 10;

// motor encoder data pins
int motorEncoderAHigh = 2;
int motorEncoderALow = 3;

int motorEncoderBHigh = 5;
int motorEncoderBLow = 6;

// motor turn off
int motorTogglePin = 4;

void setup() {
  // put your setup code here, to run once:
  pinMode(motorSwitchPin1, OUTPUT);
  pinMode(motorSwitchPin2, OUTPUT);
  
  pinMode(motorVoltagePin1, OUTPUT);
  pinMode(motorVoltagePin2, OUTPUT);

  pinMode(motorTogglePin, OUTPUT);
  
  // turn motors on
  digitalWrite(motorTogglePin, HIGH);

  digitalWrite(motorSwitchPin1, HIGH);
  digitalWrite(motorSwitchPin2, LOW);

  Serial.begin(115200);
}

void loop() {
  // fade in from min to max in increments of 5 points:
  for (int fadeValue = 0; fadeValue <= 255; fadeValue += 5) {
    // sets the value (range from 0 to 255):
    analogWrite(motorVoltagePin1, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }

  // fade out from max to min in increments of 5 points:
  for (int fadeValue = 255; fadeValue >= 0; fadeValue -= 5) {
    // sets the value (range from 0 to 255):
    analogWrite(motorVoltagePin1, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }
}

int thisA;

void aPinInterrupt() {
  thisA = digitalRead(motorEncoderAHigh);

  if (thisA == HIGH && lastA == LOW) {
    // logic for CW and CCW rotations
    if (digitalRead(BPIN) == HIGH) {
      clockwise();
    }
    else {
      counterClockwise();
    }
  }

  // save current state of A
  lastA = thisA;
}

void clockwise() {
  count += 4;
  Serial.print("CW ");
  Serial.println(count);
}

void counterClockwise() {
  count -= 4;
  Serial.print("CCW ");
  Serial.println(count);
}
