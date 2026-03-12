#pragma once

#include <WiFiUdp.h>

#include "core/Types.h"

class ManualController {
public:
    void begin(uint16_t commandPort);
    ManualCommand poll();

private:
    static bool parsePair(const String& key, const String& value, ManualCommand& command);

    WiFiUDP _udp;
};
