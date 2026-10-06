//const int IR_PIN = 2;
//onst int IR_PIN = 4;
//const int IR_PIN = 7;
const int IR_PIN = 8;

void setup() {
  Serial.begin(9600);
  pinMode(IR_PIN, INPUT);
}

void loop() {
  int state = digitalRead(IR_PIN);

  if (state == LOW) {
    Serial.println("Obstacle detected");
  } else {
    Serial.println("Clear");
  }
  delay(200);
}