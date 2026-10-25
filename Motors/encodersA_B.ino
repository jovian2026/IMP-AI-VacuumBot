volatile long leftTicks = 0;
volatile long rightTicks = 0;

const int L_A = 2, L_B = 4; //(green,yello) mirror
const int R_A = 3, R_B = 5; //(yeelow,gree)

// Flip to -1 if a wheel counts down when turned forward
const int L_DIR = 1;
const int R_DIR = 1;

void leftISR() {
  if (digitalRead(L_B) == HIGH) leftTicks += L_DIR;
  else leftTicks -= L_DIR;
}

void rightISR() {
  if (digitalRead(R_B) == HIGH) rightTicks += R_DIR;
  else rightTicks -= R_DIR;
}

void setup() {
  Serial.begin(115200);
  pinMode(L_A, INPUT_PULLUP);
  pinMode(L_B, INPUT_PULLUP);
  pinMode(R_A, INPUT_PULLUP);
  pinMode(R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_A), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(R_A), rightISR, RISING);
  Serial.println("Send 'r' to reset both counts");
}

void loop() {
  if (Serial.available() && Serial.read() == 'r') {
    noInterrupts();
    leftTicks = 0;
    rightTicks = 0;
    interrupts();
  }

  noInterrupts();
  long l = leftTicks;
  long r = rightTicks;
  interrupts();

  Serial.print("L: ");
  Serial.print(l);
  Serial.print("   R: ");
  Serial.println(r);

  delay(100);
}