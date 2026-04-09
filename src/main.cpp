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

static const uint8_t REED_SWITCH_PIN = 18;
Bounce2::Button reedSwitch = Bounce2::Button();

// Buzzer Pin
static const uint8_t BUZZER_PIN = 8;
double heading = 0.0;

// ESC timing
static const int PWM_FREQ = 50;
static const int PWM_RES_BITS = 10;

// Motors
Motor bottomLeftMotor(11, 0, PWM_FREQ, PWM_RES_BITS, 1.0f);
Motor bottomRightMotor(10, 1, PWM_FREQ, PWM_RES_BITS, 1.0f);
Motor topLeftMotor(12, 2, PWM_FREQ, PWM_RES_BITS, 1.0f);
Motor topRightMotor(9, 3, PWM_FREQ, PWM_RES_BITS, 1.0f);

// IMU
static const uint8_t BNO08X_SCK = 7;
static const uint8_t BNO08X_MISO = 15;
static const uint8_t BNO08X_MOSI = 5;
static const uint8_t BNO08X_CS = 4;
static const uint8_t BNO08X_INT = 16;
static const int8_t BNO08X_RST = 6;

Adafruit_BNO08x bno085(BNO08X_RST);

double yaw, pitch, roll;
sh2_SensorValue_t sensorValue;

// Timing
static const double pathTime = 10000;
unsigned long timer = 0;

// PIDs
static double baseSpeed = 0.8;

// Yaw PID
double yawInput, yawOutput, yawSetpoint;

double yawkp = 0.0;
double yawki = 0.0;
double yawkd = 0.0;
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, yawkp, yawki, yawkd, DIRECT);

// Pitch PID
double pitchInput, pitchOutput, pitchSetpoint = -30;
double pitchkp = 0.8;
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

void updatePID() {
    yawPID.Compute();
    pitchPID.Compute();
    rollPID.Compute();
}

void armedBeep() {
    static unsigned long lastToggle = 0;
    static bool buzzerOn = false;
    unsigned long now = millis();
    if (now - lastToggle >= (buzzerOn ? 50UL : 800UL)) {
        buzzerOn = !buzzerOn;
        digitalWrite(BUZZER_PIN, buzzerOn ? HIGH : LOW);
        lastToggle = now;
    }
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

    pitchInput = -pitch;
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

void setup() {
    bottomLeftMotor.begin();
    bottomRightMotor.begin();
    topLeftMotor.begin();
    topRightMotor.begin();

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

    rollPID.SetOutputLimits(-0.2, 0.2);
    pitchPID.SetOutputLimits(-0.2, 0.2);
    yawPID.SetOutputLimits(-0.2, 0.2);

    //Pin configurations
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    reedSwitch.attach(REED_SWITCH_PIN, INPUT_PULLUP);
    reedSwitch.interval(500);
    reedSwitch.setPressedState(false);

    Serial.println("Setup complete.");
}

void loop() {

    reedSwitch.update();

    ArduinoOTA.handle();

    switch (runState) {

        case RunState::Idle:
            if (!wifiConnected) connectToWiFi();
            if (reedSwitch.isPressed()) {
                Serial.println(">>> Transitioning Idle -> Armed");
                runState = RunState::Armed;
            }
            break;

        case RunState::Armed:
            WiFi.disconnect();
            wifiConnected = false;
            updateIMU();
            armedBeep();
            if (reedSwitch.released()) {
                if (yaw < 0) yaw += 360.0;
                heading = yaw;
                Serial.printf(">>> Transitioning Armed -> Running | Captured heading: %.1f\n", heading);
                digitalWrite(BUZZER_PIN, LOW);
                timer = millis() + pathTime;
                runState = RunState::Running;
            }
            break;

        case RunState::Running:
            if (millis() <= timer) {
                path();
            } else {
                Serial.println(">>> Transitioning Running -> Finished");
                runState = RunState::Finished;
            }
            break;

        case RunState::Finished:
            stopMotors();
            Serial.println(">>> Transitioning Finished -> Idle");
            runState = RunState::Idle;
            break;
    }
}