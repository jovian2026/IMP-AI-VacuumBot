// Motor pins
const int ENA = 5, IN1 = 4, IN2 = 7;     // left motor
const int ENB = 6, IN3 = 8, IN4 = 12;    // right motor
// Encoder pins
const int L_A = 2, L_B = 9;   // wires green, yellow
const int R_A = 3, R_B = 10; // wires yellow, green

const int L_ENC_DIR = 1, R_ENC_DIR = 1;
const bool L_MOTOR_INV = false, R_MOTOR_INV = false;

volatile long leftTicks = 0, rightTicks = 0;
unsigned long lastCmd = 0, lastSend = 0;
bool moving = false;
char buf[32];
byte n = 0;

void leftISR()  { leftTicks  += (digitalRead(L_B) == HIGH ? 1 : -1) * L_ENC_DIR; }
void rightISR() { rightTicks += (digitalRead(R_B) == HIGH ? 1 : -1) * R_ENC_DIR; }

void setMotor(int inA, int inB, int en, int spd, bool inv) {
  if (inv) spd = -spd;
  if (spd > 0)      { digitalWrite(inA, HIGH); digitalWrite(inB, LOW);  analogWrite(en, spd); }
  else if (spd < 0) { digitalWrite(inA, LOW);  digitalWrite(inB, HIGH); analogWrite(en, -spd); }
  else              { digitalWrite(inA, LOW);  digitalWrite(inB, LOW);  analogWrite(en, 0); }
}

void drive(int l, int r) {
  setMotor(IN1, IN2, ENA, l, L_MOTOR_INV);
  setMotor(IN3, IN4, ENB, r, R_MOTOR_INV);
}

void brake() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH); analogWrite(ENA, 255);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH); analogWrite(ENB, 255);
  delay(150);
  drive(0, 0);
}

void handleLine(char *s) {
  if (s[0] == 'M') {
    int l = 0, r = 0;
    if (sscanf(s, "M,%d,%d", &l, &r) == 2) {
      l = constrain(l, -255, 255);
      r = constrain(r, -255, 255);
      drive(l, r);
      moving = (l != 0 || r != 0);
      lastCmd = millis();
    }
  } else if (s[0] == 'B') {
    brake(); moving = false;
  } else if (s[0] == 'R') {
    noInterrupts(); leftTicks = 0; rightTicks = 0; interrupts();
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { buf[n] = 0; handleLine(buf); n = 0; }
    else if (c != '\r' && n < sizeof(buf) - 1) buf[n++] = c;
  }
}

void setup() {
  Serial.begin(115200);
  int outs[] = {ENA, IN1, IN2, ENB, IN3, IN4};
  for (int p : outs) pinMode(p, OUTPUT);
  pinMode(L_A, INPUT_PULLUP); pinMode(L_B, INPUT_PULLUP);
  pinMode(R_A, INPUT_PULLUP); pinMode(R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_A), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(R_A), rightISR, RISING);
  drive(0, 0);
}

void loop() {
  readSerial();

  // Watchdog: stop if the Pi goes quiet
  if (moving && millis() - lastCmd > 500) { brake(); moving = false; }

  if (millis() - lastSend >= 50) {
    lastSend = millis();
    noInterrupts(); long l = leftTicks, r = rightTicks; interrupts();
    Serial.print("E,"); Serial.print(millis()); Serial.print(',');
    Serial.print(l);   Serial.print(',');       Serial.println(r);
  }
}