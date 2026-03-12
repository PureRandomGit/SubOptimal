#include "control/AutonomousControl.h"

#include <Arduino.h>

#include "subsystems/IMUSubsystem.h"

namespace {
constexpr float BASE_THRUST = 0.35f;
constexpr float KP_YAW = 0.006f;
constexpr float MAX_CORRECTION = 0.20f;
}

void AutonomousController::begin() {
    _targetInitialized = false;
}

void AutonomousController::setTargetYaw(float yawDeg) {
    _targetYawDeg = normalizeAngle(yawDeg);
    _targetInitialized = true;
}

MotorSpeeds AutonomousController::update(const IMUSubsystem& imu) {
    MotorSpeeds out;

    if (!imu.isHealthy()) {
        return out;
    }

    if (!_targetInitialized) {
        setTargetYaw(imu.yawDeg());
    }

    const float error = normalizeAngle(_targetYawDeg - imu.yawDeg());
    const float correction = constrain(KP_YAW * error, -MAX_CORRECTION, MAX_CORRECTION);

    const float left = constrain(BASE_THRUST + correction, 0.0f, 1.0f);
    const float right = constrain(BASE_THRUST - correction, 0.0f, 1.0f);

    out.bl = left;
    out.tl = left;
    out.br = right;
    out.tr = right;
    return out;
}

float AutonomousController::normalizeAngle(float angleDeg) {
    while (angleDeg > 180.0f) {
        angleDeg -= 360.0f;
    }
    while (angleDeg < -180.0f) {
        angleDeg += 360.0f;
    }
    return angleDeg;
}
