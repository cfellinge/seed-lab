const int LED_pin = 13; 
volatile byte count;
byte reload = 0x9C; 

ISR(TIMER2_COMPA_vect)
{
  count++;
  OCR2A = reload;
}

void setup()
{
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
  Serial.print("OCR2A: "); 
  Serial.println(OCR2A, HEX);
  Serial.print("TCCR2A: "); 
  Serial.println(TCCR2A, HEX);
  Serial.print("TCCR2B: ");
  Serial.println(TCCR2B, HEX);
  Serial.print("TIMSK2: "); 
  Serial.println(TIMSK2, HEX);
  Serial.println("TIMER2 Setup Finished.");
}
void loop()
{
if (count == 50)
{
flash();
Serial.print(".");
count = 0;
}
}
void flash()
{
static boolean output = HIGH;
digitalWrite(LED_pin, output);
output = !output;
}