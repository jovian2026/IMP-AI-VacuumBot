// Two DC motors on an L298N
const int ENA = 9, IN1 = 7, IN2 = 8;    // Motor A
const int ENB = 10, IN3 = 5, IN4 = 4;   // Motor B
const int MAX_DUTY = 80;                // % cap; raise to 100 if motors are 12V

void setSpeedA(int p) { analogWrite(ENA, map(constrain(p, 0, MAX_DUTY), 0, 100, 0, 255)); }
void setSpeedB(int p) { analogWrite(ENB, map(constrain(p, 0, MAX_DUTY), 0, 100, 0, 255)); }

void forwardA()  { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
void backwardA() { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
void stopA()     { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW); setSpeedA(0); }

void forwardB()  { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
void backwardB() { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
void stopB()     { digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW); setSpeedB(0); }

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopA(); stopB();
  Serial.begin(9600);
}

void loop() {
  Serial.println("Motor A forward 40 %");
  forwardA(); setSpeedA(40); delay(3000);
  stopA(); delay(1000);

  Serial.println("Motor B forward 40 %");
  forwardB(); setSpeedB(40); delay(3000);
  stopB(); delay(1000);

  Serial.println("Both forward 40 %");
  forwardA(); forwardB(); setSpeedA(40); setSpeedB(40); delay(3000);

  Serial.println("Both forward 50 %");
  setSpeedA(50); setSpeedB(50); delay(3000);
  stopA(); stopB(); delay(2000);

  Serial.println("Both backward 40 %");
  backwardA(); backwardB(); setSpeedA(40); setSpeedB(40); delay(3000);
  stopA(); stopB(); delay(2000);
}