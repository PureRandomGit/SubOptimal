#include "Sub.h"

#include <Wire.h>

#include "core/Config.h"

void Sub::initWiFi() {
    WiFi.mode(WIFI_MODE_STA);
    WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);

    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print('.');
        delay(500);
    }
    Serial.println();
    Serial.print("WiFi connected: ");
    Serial.println(WiFi.localIP());
}

void Sub::init() {
    initWiFi();

    drive.begin();
    imu.begin(Wire, Config::I2C_SDA_PIN, Config::I2C_SCL_PIN, Config::BNO08X_INT_PIN, Config::BNO08X_RST_PIN);
    manualCtrl.begin(Config::COMMAND_PORT);
    autoCtrl.begin();
    telemetry.begin(telemetryUdp, Config::TELEMETRY_TARGET_IP, Config::TELEMETRY_PORT);
}

void Sub::update() {
    imu.update();
    drive.update();

    ManualCommand command = manualCtrl.poll();
    if (command.hasMode) {
        mode = command.mode;
    }

    if (mode == RobotMode::MANUAL) {
        if (command.hasSpeeds) {
            drive.setSpeeds(command.speeds);
        }
    } else {
        drive.setSpeeds(autoCtrl.update(imu));
    }

    telemetry.send(mode, drive.getSpeeds(), imu);
}
