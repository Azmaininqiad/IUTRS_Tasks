// Base Station Arduino Code
const int LED_PIN = 13;
String receivedData = "";

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    receivedData = Serial.readStringUntil('\n');
    receivedData.trim();

    if (receivedData == "dark") {
      Serial.println("ON");
      digitalWrite(LED_PIN, HIGH);
    } 
    else if (receivedData == "light") {
      Serial.println("OFF");
      digitalWrite(LED_PIN, LOW);
    }
  }
}