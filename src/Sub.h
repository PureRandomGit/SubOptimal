#pragma once

#include "subsystems/DriveSubsystem.h"
#include "subsystems/IMUSubsystem.h"
#include "subsystems/TelemetrySubsystem.h"
#include "control/ManualControl.h"
#include "control/AutonomousControl.h"

enum class RobotMode { MANUAL, AUTO };

class Robot {
public:
    void init();
    void update();

    RobotMode mode = RobotMode::MANUAL;

    DriveSubsystem drive;
    IMUSubsystem imu;
    TelemetrySubsystem telemetry;

    ManualController manualCtrl;
    AutonomousController autoCtrl;
};