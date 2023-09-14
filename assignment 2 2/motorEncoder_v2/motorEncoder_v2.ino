int count;
int lastCount;
byte reload = 0x9C; 
static boolean output = HIGH;

ISR(TIMER2_COMPA_vect)
{
  count++;
  OCR2A = reload;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_pin, OUTPUT);
  digitalWrite(LED_pin, LOW);
  cli();
  TCCR0B = 0; 
  OCR2A = reload;
  TCCR2A = 1<<WGM21;
  TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20);
  TIMSK2 = (1<<OCIE2A);
  sei();

}

void loop() {
  if (lastCount != count) {
    if (count % 50 == 0) {
      flash();
      // Serial.println("LED Blink!");
    }

    if (count % 100 == 0) {
      Serial.println("1 second has passed");
    }

    if (count == 1000) {
      count = 0;
      Serial.println("10 seconds have passed");
    }

    lastCount = count;
  } 
}

void flash() {
  digitalWrite(LED_BUILTIN, output);
  output = !output;
}