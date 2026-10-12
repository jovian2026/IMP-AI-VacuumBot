const int PWM_PIN = 9;
const int STEPS[] = {0, 30, 50, 65, 80};
const int NUM_STEPS = sizeof(STEPS) / sizeof(STEPS[0]);
const unsigned long HOLD_MS = 3000;

void setDuty(int percent) {
  OCR1A = (long)ICR1 * percent / 100;
}

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  ICR1 = 1066;                       // ~15 kHz
  Serial.begin(9600);

  setDuty(50);                       // start-up signal
  Serial.println("Holding 50 % - switch on 12V now");
  delay(5000);
}

void loop() {
  for (int i = 0; i < NUM_STEPS; i++) {
    setDuty(STEPS[i]);
    Serial.print("Duty: ");
    Serial.print(STEPS[i]);
    Serial.println(" %");
    delay(HOLD_MS);
  }
}