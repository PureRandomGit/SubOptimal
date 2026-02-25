#include <Arduino.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <Wire.h>
#include "Motor.h"

BNO08x imu;

Motor bottomLeftMotor(9, 0);
Motor bottomRightMotor(10, 1);
Motor topLeftMotor(11, 2);
Motor topRightMotor(12, 3);

boolean calibrate = false;

int pwmFreq = 1000;
const int pwmResBits = 10;

const int reedSwitchPin = 37; // TODO: Confirm Pins

// BNO085 Pins
const int BNO08X_INT  = 8;
const int BNO08X_RST = 9;
const int I2C_SDA1 = 25;
const int I2C_SCL1 = 27;

void calibrateESC() {
    // Print Left Motor calibrating
    Serial.println("Calibrating Motor");
    bottomLeftMotor.setSpeed(1.0);
    bottomRightMotor.setSpeed(1.0);
    topLeftMotor.setSpeed(1.0);
    topRightMotor.setSpeed(1.0);

    delay(8000);

    Serial.println("Stopping Motor");
    bottomLeftMotor.stop();
    bottomRightMotor.stop();
    topLeftMotor.stop();
    topRightMotor.stop();

    delay(8000);
    Serial.println("Calibration Complete");
}

void setup() {
    Serial.begin(115200);

    bottomLeftMotor.begin();
    bottomRightMotor.begin();
    topLeftMotor.begin();
    topRightMotor.begin();

    delay(2000);

    if (calibrate) {
        calibrateESC();
    }
}

void loop() {
    Serial.println("Reving Motors");

    delay(1000);

    for (float i = 0; i < 1; i = i + 0.01) {
        Serial.println(i);
        bottomLeftMotor.setSpeed(i);
        bottomRightMotor.setSpeed(i);
        topLeftMotor.setSpeed(i);
        topRightMotor.setSpeed(i);
        Serial.println(bottomLeftMotor.getSpeed());
        delay(100);
    }

    Serial.println("Slowing Motors");
    delay(1000);

    for (float i = 1; i > 0; i = i - 0.01) {
        bottomLeftMotor.setSpeed(i);
        bottomRightMotor.setSpeed(i);
        topLeftMotor.setSpeed(i);
        topRightMotor.setSpeed(i);
        Serial.println(bottomLeftMotor.getSpeed());
        delay(100);
    }

    bottomLeftMotor.stop();
    delay(5000);
}