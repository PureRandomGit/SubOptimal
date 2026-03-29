#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <SPI.h>
#include <PID_v1.h>
#include <Adafruit_BNO08x.h>
#include <Bounce2.h>
#include "Motor.h"


// WiFi
static const char* WIFI_SSID = "Suboptimal";
static const char* WIFI_PASS = "Suboptimal123";
bool wifiConnected = false;

// Reed switch: magnet held = closed/LOW, release = start mission
static const uint8_t REED_SWITCH_PIN = 4;
Bounce2::Button reedSwitch = Bounce2::Button();

// Buzzer Pin
static const uint8_t BUZZER_PIN = 5;
static const double heading = 30.0; // Desired heading in degrees

// ESC timing
static const int PWM_FREQ = 50;
static const int PWM_RES_BITS = 10;

// Motors
Motor bottomLeftMotor(9, 0, PWM_FREQ, PWM_RES_BITS, 1.0f);
Motor bottomRightMotor(10, 1, PWM_FREQ, PWM_RES_BITS, 1.0f);
Motor topLeftMotor(11, 2, PWM_FREQ, PWM_RES_BITS, 1.0f);
Motor topRightMotor(12, 3, PWM_FREQ, PWM_RES_BITS, 1.0f);

// IMU
Adafruit_BNO08x bno085(BNO08X_RST);

static const uint8_t BNO08X_SCK = 18;
static const uint8_t BNO08X_MISO = 17;
static const uint8_t BNO08X_MOSI = 16;
static const uint8_t BNO08X_CS = 15;
static const uint8_t BNO08X_INT = 14;
static const int8_t BNO08X_RST = 13;

double yaw, pitch, roll;
sh2_SensorValue_t sensorValue;

// Timing
static const double pathTime = 5000; // 5 seconds
unsigned long timer = 0;

// PIDs

static double baseSpeed = 0.5;

// Yaw PID
double yawInput, yawOutput, yawSetpoint;

double yawkp = 0.0;
double yawki = 0.0;
double yawkd = 0.0;
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, yawkp, yawki, yawkd, DIRECT);

// Pitch PID
double pitchInput, pitchOutput, pitchSetpoint;
double pitchkp = 0.0;
double pitchki = 0.0;
double pitchkd = 0.0;
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, pitchkp, pitchki, pitchkd, DIRECT);

// Roll PID
double rollInput, rollOutput, rollSetpoint;
double rollkp = 0.0;
double rollki = 0.0;
double rollkd = 0.0;
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, rollkp, rollki, rollkd, DIRECT);

enum class RunState {
    Idle,
    Armed,
    Running,
    Finished
};

RunState runState = RunState::Idle;

// Connects To Wifi
void connectToWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.println("\nConnecting to WiFi Network ..");

    while(WiFi.status() != WL_CONNECTED){
        Serial.print(".");
        delay(100);
    }
    Serial.println("\nConnected to the WiFi network");
    Serial.print("Local ESP32 IP: ");
    Serial.println(WiFi.localIP());

    ArduinoOTA.begin();

    wifiConnected = true;
}

float toDegrees(double radians) {
    return radians * 180.0 / M_PI;
}

void stopMotors() {
    bottomLeftMotor.stop();
    bottomRightMotor.stop();
    topLeftMotor.stop();
    topRightMotor.stop();
}

void headingBeep() {
    updateIMU();

    if (yaw < 0) yaw += 360.0;

    double diff = abs(yaw - heading);
    if (diff > 180.0) diff = 360.0 - diff;  // shortest angular distance

    unsigned long interval = (unsigned long)map((long)diff, 0, 180, 100, 1500);

    static unsigned long lastBeep = 0;
    unsigned long now = millis();
    if (now - lastBeep >= interval) {
        tone(BUZZER_PIN, 1000, 50);
        lastBeep = now;
    }
}



void updatePID() {
    yawPID.Compute();
    pitchPID.Compute();
    rollPID.Compute();
}

double calculateError(double current, double target) {
    double error = target - current;
    if (error > 180.0) error -= 360.0;
    if (error < -180.0) error += 360.0;
    return error;
}

void path() {
    updateIMU();

    if (yaw < 0) yaw += 360.0;

    double yawError = -calculateError(yaw, heading);

    pitchInput = pitch;
    yawInput = yawError;
    rollInput = roll;

    updatePID();

    float tl = baseSpeed + pitchOutput + yawOutput + rollOutput;
    float tr = baseSpeed + pitchOutput - yawOutput - rollOutput;
    float bl = baseSpeed - pitchOutput + yawOutput - rollOutput;
    float br = baseSpeed - pitchOutput - yawOutput + rollOutput;

    bottomLeftMotor.setSpeed(bl);
    bottomRightMotor.setSpeed(br);
    topLeftMotor.setSpeed(tl);

    topRightMotor.setSpeed(tr);
}

void updateIMU() {
    if (bno085.getSensorEvent(&sensorValue)) {
        float qw = sensorValue.un.rotationVector.real;
        float qx = sensorValue.un.rotationVector.i;
        float qy = sensorValue.un.rotationVector.j;
        float qz = sensorValue.un.rotationVector.k;

        yaw   = toDegrees(atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)));
        pitch = toDegrees(asin(2.0 * (qw * qy - qz * qx)));
        roll  = toDegrees(atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)));
    }
}

void setup() {
    delay(1000);
    Serial.begin(115200);
    connectToWiFi();

    // IMU Setup
    SPI.begin(BNO08X_SCK, BNO08X_MISO, BNO08X_MOSI, BNO08X_CS);
    if (!bno085.begin_SPI(BNO08X_CS, BNO08X_INT)) {
        Serial.println("Failed to initialize BNO08x!");
    }

    bno085.enableReport(SH2_ROTATION_VECTOR, 10000);

    // PID Setup
    rollPID.SetMode(AUTOMATIC);
    pitchPID.SetMode(AUTOMATIC);
    yawPID.SetMode(AUTOMATIC);

    rollPID.SetOutputLimits(-0.1, 0.1);
    pitchPID.SetOutputLimits(-0.1, 0.1);
    yawPID.SetOutputLimits(-0.1, 0.1);

    // Motor Setup
    bottomLeftMotor.begin();
    bottomRightMotor.begin();
    topLeftMotor.begin();
    topRightMotor.begin();

    //Pin configiurations
    reedSwitch.attach(REED_SWITCH_PIN, INPUT_PULLUP);
    reedSwitch.interval(5);
    reedSwitch.setPressedState(HIGH); // TODO: Test if this is the correct state

}

void loop() {

    reedSwitch.update();

    ArduinoOTA.handle();  // Handles OTA

    switch (runState) {

        case RunState::Idle:
            if (!wifiConnected) connectToWiFi();
            if (reedSwitch.pressed()) runState = RunState::Armed;
            break;

        case RunState::Armed:
            WiFi.disconnect();
            wifiConnected = false;
            headingBeep();
            if (reedSwitch.released()) {
                timer = millis() + pathTime;
                runState = RunState::Running;
            }
            break;

        case RunState::Running:
            if (millis() <= timer) {
                path();
            } else {
                runState = RunState::Finished;
            }
            break;

        case RunState::Finished:
            stopMotors();
            runState = RunState::Idle;
            break;
    }
}