#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "driver/ledc.h"

class Motor {
private:
    int pin;
    int channel;
    int frequency;
    int resolutionBits;
    float currentSpeed;
    int maxDuty;
public:
    Motor(int pin, int channel, int frequency = 50, int resolutionBits = 10);

    void begin();
    void setSpeed(float speed);
    void stop();
    float getSpeed();
};

#endif