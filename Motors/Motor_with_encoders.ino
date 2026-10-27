// Motor pins (from your wiring)
const int ENA = 5, IN1 = 4, IN2 = 7;     // left motor
const int ENB = 6, IN3 = 8, IN4 = 12;    // right motor

// Encoder pins
const int L_A = 2, L_B = 9;   //(green, yellow )
const int R_A = 3, R_B = 10;  // (yellow, green)

// Flip these if needed
const int L_ENC_DIR = 1;         // -1 if left count goes down 
const int R_ENC_DIR = 1;         // -1 if right count goes down 
const bool L_MOTOR_INV = false;  // true if the left wheel spins backward on "forward"
const bool R_MOTOR_INV = false;  // true if the right wheel spins backward on "forward"

volatile long leftTicks = 0, rightTicks = 0;
int speedPWM = 120;              // 0-255, start low
bool runFwd = false, runSpin = false;
long targetTicks = 0;
unsigned long lastCmd = 0;

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
  lastCmd = millis();
}

// NEW: actively brake instead of coasting
void brake() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH); analogWrite(ENA, 255);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH); analogWrite(ENB, 255);
  delay(150);
  drive(0, 0);
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
  Serial.println("w=fwd s=back a=spin left d=spin right x=stop g=1m q=spin +/-=speed r=reset counts");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'w': drive( speedPWM,  speedPWM); break;
      case 's': drive(-speedPWM, -speedPWM); break;
      case 'a': drive(-speedPWM,  speedPWM); break;
      case 'd': drive( speedPWM, -speedPWM); break;
      case 'x': runFwd = false; runSpin = false; brake(); break;   // CHANGED
      case '+': speedPWM = min(255, speedPWM + 20); break;
      case '-': speedPWM = max(60,  speedPWM - 20); break;
      case 'r': noInterrupts(); leftTicks = 0; rightTicks = 0; interrupts(); break;
      case 'g':  // drive 1 m forward
        noInterrupts(); leftTicks = 0; rightTicks = 0; interrupts();
        targetTicks = 4207;                  // 1000 mm / 0.2377 mm per tick
        runFwd = true; runSpin = false;
        drive(speedPWM, speedPWM);
        break;
      case 'q':  // spin left one "360" by the numbers
        noInterrupts(); leftTicks = 0; rightTicks = 0; interrupts();
        targetTicks = 2445;                  // pi x 185 mm / 0.2377 mm per tick
        runSpin = true; runFwd = false;
        drive(-speedPWM, speedPWM);
        break;
    }
  }

  if (runFwd || runSpin) {
    noInterrupts(); long l = leftTicks, r = rightTicks; interrupts();
    long done = runFwd ? (l + r) / 2 : (r - l) / 2;
    if (done >= targetTicks) { runFwd = false; runSpin = false; brake(); }   // CHANGED
    else lastCmd = millis();             // keep the safety timeout from firing
  }

  // Safety: stop if no command for 1.5 s
  if (millis() - lastCmd > 1500) drive(0, 0);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    noInterrupts(); long l = leftTicks, r = rightTicks; interrupts();
    Serial.print("L: "); Serial.print(l);
    Serial.print("   R: "); Serial.print(r);
    Serial.print("   PWM: "); Serial.println(speedPWM);
  }
}