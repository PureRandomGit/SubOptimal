#pragma once

#include "core/Types.h"

class IMUSubsystem;

class AutonomousController {
public:
    void begin();
    void setTargetYaw(float yawDeg);
    MotorSpeeds update(const IMUSubsystem& imu);

private:
    static float normalizeAngle(float angleDeg);

    float _targetYawDeg = 0.0f;
    bool _targetInitialized = false;
};
