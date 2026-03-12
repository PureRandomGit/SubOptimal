#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "driver/ledc.h"

class Motor {
private:
    uint8_t pin;
    uint8_t channel;
    uint16_t frequency;
    uint8_t resolutionBits;
    float currentSpeed;
    uint32_t maxDuty;
public:
    Motor(uint8_t pin, uint8_t channel, uint16_t frequency = 50, uint8_t resolutionBits = 12);

    void begin();
    void setSpeed(float speed);
    void stop();
    float getSpeed() const;
};

#endif