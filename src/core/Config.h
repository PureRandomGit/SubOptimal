#pragma once

#include <Arduino.h>

namespace Config {
constexpr uint8_t MOTOR_BL_PIN = 9;
constexpr uint8_t MOTOR_BR_PIN = 10;
constexpr uint8_t MOTOR_TL_PIN = 11;
constexpr uint8_t MOTOR_TR_PIN = 12;

constexpr uint8_t MOTOR_BL_CHANNEL = 0;
constexpr uint8_t MOTOR_BR_CHANNEL = 1;
constexpr uint8_t MOTOR_TL_CHANNEL = 2;
constexpr uint8_t MOTOR_TR_CHANNEL = 3;

constexpr int BNO08X_INT_PIN = 15;
constexpr int BNO08X_RST_PIN = 16;
constexpr int I2C_SDA_PIN = 17;
constexpr int I2C_SCL_PIN = 18;

constexpr uint16_t TELEMETRY_PORT = 4444;
constexpr uint16_t COMMAND_PORT = 4445;

constexpr const char* WIFI_SSID = "SubOptimal";
constexpr const char* WIFI_PASSWORD = "Password123!@#";

const IPAddress TELEMETRY_TARGET_IP(192, 168, 0, 255);
}  // namespace Config
