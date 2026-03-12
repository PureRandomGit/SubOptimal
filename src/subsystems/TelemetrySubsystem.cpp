#include "subsystems/TelemetrySubsystem.h"

#include "subsystems/IMUSubsystem.h"

void TelemetrySubsystem::begin(WiFiUDP& udp, const IPAddress& targetIp, uint16_t targetPort) {
    _udp = &udp;
    _targetIp = targetIp;
    _targetPort = targetPort;
    _udp->begin(targetPort);
}

void TelemetrySubsystem::send(RobotMode mode, const MotorSpeeds& speeds, const IMUSubsystem& imu) {
    if (_udp == nullptr) {
        return;
    }

    if (millis() - _lastTxMs < TX_INTERVAL_MS) {
        return;
    }

    char payload[256];
    snprintf(payload,
             sizeof(payload),
             "{\"mode\":\"%s\",\"imu_ok\":%s,\"yaw\":%.2f,\"pitch\":%.2f,\"roll\":%.2f,\"bl\":%.3f,\"br\":%.3f,\"tl\":%.3f,\"tr\":%.3f}",
             mode == RobotMode::MANUAL ? "manual" : "auto",
             imu.isHealthy() ? "true" : "false",
             imu.yawDeg(),
             imu.pitchDeg(),
             imu.rollDeg(),
             speeds.bl,
             speeds.br,
             speeds.tl,
             speeds.tr);

    _udp->beginPacket(_targetIp, _targetPort);
    _udp->write(reinterpret_cast<const uint8_t*>(payload), strlen(payload));
    _udp->endPacket();

    _lastTxMs = millis();
}
