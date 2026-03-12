#pragma once

#include <WiFi.h>
#include <WiFiUdp.h>

#include "core/Types.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IMUSubsystem.h"
#include "subsystems/TelemetrySubsystem.h"
#include "control/ManualControl.h"
#include "control/AutonomousControl.h"

class Sub {
public:
    void init();
    void update();

    RobotMode mode = RobotMode::MANUAL;

    DriveSubsystem drive;
    IMUSubsystem imu;
    TelemetrySubsystem telemetry;

    ManualController manualCtrl;
    AutonomousController autoCtrl;

private:
    void initWiFi();

    WiFiUDP telemetryUdp;
};