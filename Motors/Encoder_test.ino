volatile long ticks = 0;
const int ENC_A = 2, ENC_B = 4; // Right encoder
//const int ENC_A =12, ENC_B = 8; // Left encoder

void isr() {
  if (digitalRead(ENC_B) == HIGH) ticks++;
  else ticks--;
}

void setup() {
  Serial.begin(115200);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isr, RISING);
}

void loop() {
  noInterrupts();
  long t = ticks;
  interrupts();
  Serial.println(t);
  delay(100);
}
