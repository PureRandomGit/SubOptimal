#include <Arduino.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <Wire.h>
#include "Motor.h"
#include <WiFi.h>
#include <WiFiUdp.h>

BNO08x imu;

Motor bottomLeftMotor(9, 0);
Motor bottomRightMotor(10, 1);
Motor topLeftMotor(11, 2);
Motor topRightMotor(12, 3);

boolean calibrate = false;

const int reedSwitchPin = 37; // TODO: Confirm Pins

// BNO085 Pins
const int BNO08X_INT  = 8;
const int BNO08X_RST = 9;
const int I2C_SDA1 = 25;
const int I2C_SCL1 = 27;

// Wifi Setup
const char * ssid = "SubOptimal";
const char * password = "Password123!@#";

// UDP
const char * udpAddress = "192.168.0.255";
const int udpPort = 4444;
const int cmdPort = 4445;  // Port to receive motor commands from backend

WiFiUDP udp;
WiFiUDP cmdUdp;

void initWiFi() {
    WiFi.mode(WIFI_MODE_STA);
    WiFi.begin(ssid, password);

    Serial.print("Connecting to WiFi ..");

    while (WiFi.status() != WL_CONNECTED) {
        Serial.print('.');
        delay(1000);
    }

    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    udp.begin(udpPort);
    cmdUdp.begin(cmdPort);
}


void calibrateESC() {
    Serial.println("Motors Calibration Starting");

    delay(1000);

    bottomLeftMotor.setSpeed(1.0);
    bottomRightMotor.setSpeed(1.0);
    topLeftMotor.setSpeed(1.0);
    topRightMotor.setSpeed(1.0);

    Serial.println("Power ESCs");

    delay(8000);

    Serial.println("Stopping Motor");
    bottomLeftMotor.stop();
    bottomRightMotor.stop();
    topLeftMotor.stop();
    topRightMotor.stop();

    delay(5000);
    Serial.println("Calibration Complete");
}

void handleCommands() {
    int packetSize = cmdUdp.parsePacket();
    if (packetSize <= 0) return;

    char buf[256];
    int len = cmdUdp.read(buf, sizeof(buf) - 1);
    buf[len] = '\0';

    // Parse "key:value,key:value" pairs and apply motor speeds
    String data = String(buf);
    int start = 0;
    while (start < (int)data.length()) {
        int commaIdx = data.indexOf(',', start);
        if (commaIdx == -1) commaIdx = data.length();
        String pair = data.substring(start, commaIdx);
        int colonIdx = pair.indexOf(':');
        if (colonIdx != -1) {
            String key = pair.substring(0, colonIdx);
            float val = pair.substring(colonIdx + 1).toFloat();
            key.trim();
            if      (key == "bl") bottomLeftMotor.setSpeed(val);
            else if (key == "br") bottomRightMotor.setSpeed(val);
            else if (key == "tl") topLeftMotor.setSpeed(val);
            else if (key == "tr") topRightMotor.setSpeed(val);
        }
        start = commaIdx + 1;
    }
}

void setup() {
    initWiFi();

    delay(1000);

    Serial.begin(115200);

    // Initialize Motors
    bottomLeftMotor.begin();
    bottomRightMotor.begin();
    topLeftMotor.begin();
    topRightMotor.begin();

    if (calibrate) calibrateESC();
}

void loop() {
    handleCommands();
}

