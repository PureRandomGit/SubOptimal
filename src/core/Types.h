#pragma once

enum class RobotMode {
    MANUAL,
    AUTO
};

struct MotorSpeeds {
    float bl = 0.0f;
    float br = 0.0f;
    float tl = 0.0f;
    float tr = 0.0f;
};

struct ManualCommand {
    bool hasSpeeds = false;
    bool hasMode = false;
    MotorSpeeds speeds;
    RobotMode mode = RobotMode::MANUAL;
};
