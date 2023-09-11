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

  Serial.begin(115200);
}

int x = 100;

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(motorVoltagePin1, x);
  Serial.print(x);
  delay(5);
}
