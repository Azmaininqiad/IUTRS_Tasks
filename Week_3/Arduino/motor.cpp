#include "motor.h"


Motor::Motor(int dirPin1, int dirPin2, int enablePin) 
    : _dirPin1(dirPin1), _dirPin2(dirPin2), _enablePin(enablePin) {
}
void Motor::begin() {
    pinMode(_dirPin1, OUTPUT);
    pinMode(_dirPin2, OUTPUT);
    pinMode(_enablePin, OUTPUT);
    
    stop(); 
}

void Motor::forward(int pwmSpeed) {
    if (pwmSpeed > 255) pwmSpeed = 255;
    if (pwmSpeed < 0) pwmSpeed = 0;

    digitalWrite(_dirPin1, HIGH);
    digitalWrite(_dirPin2, LOW);
    
    analogWrite(_enablePin, pwmSpeed);
}
void Motor::backward(int pwmSpeed) {
    if (pwmSpeed > 255) pwmSpeed = 255;
    if (pwmSpeed < 0) pwmSpeed = 0;

    digitalWrite(_dirPin1, LOW);
    digitalWrite(_dirPin2, HIGH);
    
    analogWrite(_enablePin, pwmSpeed);
}

void Motor::stop() {
    analogWrite(_enablePin, 0); 

    digitalWrite(_dirPin1, LOW); 
    digitalWrite(_dirPin2, LOW); 
}
