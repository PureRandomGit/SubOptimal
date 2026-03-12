#pragma once

#include <Arduino.h>

#include "core/Config.h"
#include "core/Types.h"
#include "subsystems/Motor.h"

class DriveSubsystem {
public:
    void begin();
    void update();
    void setSpeeds(const MotorSpeeds& speeds);
    void stop();
    const MotorSpeeds& getSpeeds() const;

private:
    Motor _bl{Config::MOTOR_BL_PIN, Config::MOTOR_BL_CHANNEL};
    Motor _br{Config::MOTOR_BR_PIN, Config::MOTOR_BR_CHANNEL};
    Motor _tl{Config::MOTOR_TL_PIN, Config::MOTOR_TL_CHANNEL};
    Motor _tr{Config::MOTOR_TR_PIN, Config::MOTOR_TR_CHANNEL};
    MotorSpeeds _currentSpeeds;

    unsigned long _lastCommandTimeMs = 0;
    static constexpr unsigned long COMMAND_TIMEOUT_MS = 500;
};