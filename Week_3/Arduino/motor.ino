#include "motor.h"

#define LEFT_DIR_PIN_1 2
#define LEFT_DIR_PIN_2 3
#define LEFT_EN_PIN    9
#define RIGHT_DIR_PIN_1 4
#define RIGHT_DIR_PIN_2 5
#define RIGHT_EN_PIN    10

Motor leftMotor(LEFT_DIR_PIN_1, LEFT_DIR_PIN_2, LEFT_EN_PIN);
Motor rightMotor(RIGHT_DIR_PIN_1, RIGHT_DIR_PIN_2, RIGHT_EN_PIN);

void setup() {
  Serial.begin(9600);
  Serial.println("Motor Control Initialized.");

  leftMotor.begin();
  rightMotor.begin();
}

void loop() {
  Serial.println("Moving Forward at 200 PWM 2 seconds");
  leftMotor.forward(200);
  rightMotor.forward(200);
  delay(2000);

  Serial.println("Stopping for 1 second.");
  leftMotor.stop();
  rightMotor.stop();
  delay(1000); 

  Serial.println("Moving Backward at 150 PWM for 2 seconds");
  leftMotor.backward(150);
  rightMotor.backward(150);
  delay(2000); 

  Serial.println("Stopping for 1 second.");
  leftMotor.stop();
  rightMotor.stop();
  delay(1000); 
}
