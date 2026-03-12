#include "control/ManualControl.h"

void ManualController::begin(uint16_t commandPort) {
    _udp.begin(commandPort);
}

ManualCommand ManualController::poll() {
    ManualCommand command;

    const int packetSize = _udp.parsePacket();
    if (packetSize <= 0) {
        return command;
    }

    char buffer[256] = {};
    const int len = _udp.read(buffer, sizeof(buffer) - 1);
    if (len <= 0) {
        return command;
    }
    buffer[len] = '\0';

    String packet(buffer);
    int cursor = 0;
    while (cursor < packet.length()) {
        int commaIndex = packet.indexOf(',', cursor);
        if (commaIndex < 0) {
            commaIndex = packet.length();
        }

        String kv = packet.substring(cursor, commaIndex);
        int colonIndex = kv.indexOf(':');
        if (colonIndex > 0) {
            String key = kv.substring(0, colonIndex);
            String value = kv.substring(colonIndex + 1);
            key.trim();
            value.trim();
            parsePair(key, value, command);
        }

        cursor = commaIndex + 1;
    }

    return command;
}

bool ManualController::parsePair(const String& key, const String& value, ManualCommand& command) {
    if (key == "mode") {
        command.hasMode = true;
        command.mode = (value == "auto") ? RobotMode::AUTO : RobotMode::MANUAL;
        return true;
    }

    const float speed = constrain(value.toFloat(), 0.0f, 1.0f);
    if (key == "bl") {
        command.speeds.bl = speed;
        command.hasSpeeds = true;
        return true;
    }
    if (key == "br") {
        command.speeds.br = speed;
        command.hasSpeeds = true;
        return true;
    }
    if (key == "tl") {
        command.speeds.tl = speed;
        command.hasSpeeds = true;
        return true;
    }
    if (key == "tr") {
        command.speeds.tr = speed;
        command.hasSpeeds = true;
        return true;
    }

    return false;
}
