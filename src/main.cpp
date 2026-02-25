#include <Arduino.h>
#include <ESP32Servo.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <Wire.h>

BNO08x imu;
ESP32PWM bottomLeftPWM;
ESP32PWM bottomRightPWM;
ESP32PWM topLeftPWM;
ESP32PWM topRightPWM;

// Pin Assignemnts
const int bottomLeftMotorPin = 17; // TODO: Confirm Pins
const int bottomRightMotorPin = 18; // TODO: Confirm Pins
const int topLeftMotorPin = 19; // TODO: Confirm Pins
const int topRightMotorPin = 20; // TODO: Confirm Pins

int pwmFreq = 1000;
const int pwmResBits = 10;

const int reedSwitchPin = 37; // TODO: Confirm Pins

// BNO085 Pins
const int BNO08X_INT  = 8;
const int BNO08X_RST = 9;
const int I2C_SDA1 = 25;
const int I2C_SCL1 = 27;

void setup() {
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

	Serial.begin(115200);

	bottomLeftPWM.attachPin(bottomLeftMotorPin, pwmFreq, pwmResBits); // 1KHz 10 bits
	bottomRightPWM.attachPin(bottomRightMotorPin, pwmFreq, pwmResBits); // 1KHz 10 bits
	topLeftPWM.attachPin(topLeftMotorPin, pwmFreq, pwmResBits); // 1KHz 10 bits
	topRightPWM.attachPin(topRightMotorPin, pwmFreq, pwmResBits); // 1KHz 10 bits

  bottomLeftPWM.writeScaled(0.0); // Stop Motors
  bottomRightPWM.writeScaled(0.0); // Stop Motors
  topLeftPWM.writeScaled(0.0); // Stop Motors
  topRightPWM.writeScaled(0.0); // Stop Motors
}

void loop() {
  // put your main code here, to run repeatedly:

}

void calibrateESC() {

}