const int ledPin = 12;      // GP2Y1010AU0F Pin 3 (LED)
const int sensorPin = A0;   // GP2Y1010AU0F Pin 5 (Vo)

const int samplingTime = 280;  // 0.28ms to wait before reading
const int deltaTime = 40;      // 0.04ms to complete 0.32ms pulse
const int sleepTime = 9680;    // Remaining time in a 10ms cycle

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // Turn IR LED on (active LOW on most drive circuits)
  digitalWrite(ledPin, LOW); 
  delayMicroseconds(samplingTime);

  // Read analog value
  int rawValue = analogRead(sensorPin);

  delayMicroseconds(deltaTime);
  // Turn IR LED off
  digitalWrite(ledPin, HIGH);
  delayMicroseconds(sleepTime);

  // Convert analog reading to voltage (0 - 5V)
  float voltage = rawValue * (5.0 / 1024.0);

  Serial.print("Raw: ");
  Serial.print(rawValue);
  Serial.print(" | Voltage: ");
  Serial.println(voltage);

  delay(1000);
}