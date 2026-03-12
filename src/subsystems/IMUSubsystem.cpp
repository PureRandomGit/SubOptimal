#include "subsystems/IMUSubsystem.h"

#include <math.h>

namespace {
constexpr uint8_t BNO08X_DEFAULT_ADDR = 0x4B;
constexpr float RAD_TO_DEG_F = 57.2957795131f;
constexpr unsigned long HEALTH_TIMEOUT_MS = 1000;
}

bool IMUSubsystem::begin(TwoWire& wire, int sdaPin, int sclPin, int intPin, int rstPin) {
    wire.begin(sdaPin, sclPin);
    _online = _imu.begin(BNO08X_DEFAULT_ADDR, wire, intPin, rstPin);

    if (_online) {
        _imu.enableRotationVector(20);
        _lastSampleMs = millis();
    }

    return _online;
}

void IMUSubsystem::update() {
    if (!_online) {
        return;
    }

    while (_imu.getSensorEvent()) {
        if (_imu.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
            updateEulerFromQuaternion(_imu.getQuatI(), _imu.getQuatJ(), _imu.getQuatK(), _imu.getQuatReal());
            _lastSampleMs = millis();
        }
    }
}

float IMUSubsystem::yawDeg() const {
    return _yawDeg;
}

float IMUSubsystem::pitchDeg() const {
    return _pitchDeg;
}

float IMUSubsystem::rollDeg() const {
    return _rollDeg;
}

bool IMUSubsystem::isHealthy() const {
    return _online && (millis() - _lastSampleMs <= HEALTH_TIMEOUT_MS);
}

void IMUSubsystem::updateEulerFromQuaternion(float qi, float qj, float qk, float qr) {
    const float sinrCosp = 2.0f * (qr * qi + qj * qk);
    const float cosrCosp = 1.0f - 2.0f * (qi * qi + qj * qj);
    _rollDeg = atan2f(sinrCosp, cosrCosp) * RAD_TO_DEG_F;

    const float sinp = 2.0f * (qr * qj - qk * qi);
    if (fabsf(sinp) >= 1.0f) {
        _pitchDeg = copysignf(90.0f, sinp);
    } else {
        _pitchDeg = asinf(sinp) * RAD_TO_DEG_F;
    }

    const float sinyCosp = 2.0f * (qr * qk + qi * qj);
    const float cosyCosp = 1.0f - 2.0f * (qj * qj + qk * qk);
    _yawDeg = atan2f(sinyCosp, cosyCosp) * RAD_TO_DEG_F;
}
