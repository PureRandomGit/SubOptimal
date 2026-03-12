#include "subsystems/DriveSubsystem.h"

void DriveSubsystem::begin() {
    _bl.begin();
    _br.begin();
    _tl.begin();
    _tr.begin();

    stop();
    _lastCommandTimeMs = millis();
}

void DriveSubsystem::update() {
    if (millis() - _lastCommandTimeMs > COMMAND_TIMEOUT_MS) {
        stop();
    }
}

void DriveSubsystem::setSpeeds(const MotorSpeeds& speeds) {
    _currentSpeeds = speeds;

    _bl.setSpeed(_currentSpeeds.bl);
    _br.setSpeed(_currentSpeeds.br);
    _tl.setSpeed(_currentSpeeds.tl);
    _tr.setSpeed(_currentSpeeds.tr);

    _lastCommandTimeMs = millis();
}

void DriveSubsystem::stop() {
    _currentSpeeds = {};

    _bl.stop();
    _br.stop();
    _tl.stop();
    _tr.stop();
}

const MotorSpeeds& DriveSubsystem::getSpeeds() const {
    return _currentSpeeds;
}
