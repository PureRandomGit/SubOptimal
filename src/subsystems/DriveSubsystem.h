#pragma once
#include "Motor.h"

struct MotorSpeeds {
    float bl = 0, br = 0, tl = 0, tr = 0;
};

class DriveSubsystem {
public:
    void init();
    void update();
    void setSpeeds(const MotorSpeeds& speeds);
    void stop();

private:
    Motor _bl{9, 0}, _br{10, 1}, _tl{11, 2}, _tr{12, 3};
    MotorSpeeds _currentSpeeds;

    // Watchdog
    unsigned long _lastCommandTime = 0;
    static constexpr unsigned long TIMEOUT_MS = 500;
};