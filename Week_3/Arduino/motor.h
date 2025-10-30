#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
public:
    Motor(int dirPin1, int dirPin2, int enablePin);
    void begin();
    void forward(int pwmSpeed);
    void backward(int pwmSpeed);
    void stop();

private:
    int _dirPin1;      
    int _dirPin2;      
    int _enablePin;   
};

#endif 
