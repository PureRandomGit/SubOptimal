#pragma once

#include <Arduino.h>
#include <WiFiUdp.h>

#include "core/Types.h"

class IMUSubsystem;

class TelemetrySubsystem {
public:
    void begin(WiFiUDP& udp, const IPAddress& targetIp, uint16_t targetPort);
    void send(RobotMode mode, const MotorSpeeds& speeds, const IMUSubsystem& imu);

private:
    WiFiUDP* _udp = nullptr;
    IPAddress _targetIp;
    uint16_t _targetPort = 0;

    unsigned long _lastTxMs = 0;
    static constexpr unsigned long TX_INTERVAL_MS = 100;
};
