#pragma once

#include <Arduino.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <Wire.h>

class IMUSubsystem {
public:
    bool begin(TwoWire& wire, int sdaPin, int sclPin, int intPin, int rstPin);
    void update();

    float yawDeg() const;
    float pitchDeg() const;
    float rollDeg() const;
    bool isHealthy() const;

private:
    void updateEulerFromQuaternion(float qi, float qj, float qk, float qr);

    BNO08x _imu;
    bool _online = false;

    float _yawDeg = 0.0f;
    float _pitchDeg = 0.0f;
    float _rollDeg = 0.0f;

    unsigned long _lastSampleMs = 0;
};
