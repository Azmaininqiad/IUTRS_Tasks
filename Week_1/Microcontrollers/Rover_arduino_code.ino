// Rover Arduino Code
const int LDR_PIN = A0;
int threshold = 400; 

void setup() {
  Serial.begin(9600);
}

void loop() {
  int lightValue = analogRead(LDR_PIN);
  delay(500);
  
  if (lightValue < threshold) {
    Serial.println("dark");
  } else {
    Serial.println("light");
  }
}
