#include <Arduino.h>
#include "Sub.h"

Sub sub;

void setup() {
    Serial.begin(115200);
    sub.init();
}

void loop() {
    sub.update();
}