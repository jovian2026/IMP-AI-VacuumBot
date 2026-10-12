const int ENA = 9;   // PWM speed
const int IN1 = 7;
const int IN2 = 8;
const int MAX_DUTY = 50;   // % cap for a 6V motor on a 12V supply

void setSpeed(int percent) {
  percent = constrain(percent, 0, MAX_DUTY);
  analogWrite(ENA, map(percent, 0, 100, 0, 255));
}

void forward()  { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
void backward() { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
void stopMotor(){ digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW); setSpeed(0); }

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  Serial.println("Forward 40 %");
  forward();  setSpeed(40);  delay(3000);

  Serial.println("Forward 50 % (max)");
  setSpeed(50);              delay(3000);

  Serial.println("Stop");
  stopMotor();               delay(2000);

  Serial.println("Backward 40 %");
  backward(); setSpeed(40);  delay(3000);

  Serial.println("Stop");
  stopMotor();               delay(2000);
}