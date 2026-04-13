#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
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
Motor topLeftMotor(11, 0, PWM_FREQ, PWM_RES_BITS, 1.0f);
Motor topRightMotor(10, 1, PWM_FREQ, PWM_RES_BITS, 1.0f);
Motor bottomLeftMotor(12, 2, PWM_FREQ, PWM_RES_BITS, 1.0f);
Motor bottomRightMotor(9, 3, PWM_FREQ, PWM_RES_BITS, 1.0f);

// PID log buffer
struct LogEntry {
    uint32_t timestamp;
    float yaw, pitch, roll;
    float pitchIn, pitchOut, pitchSetpt;
    float rollIn, rollOut, rollSetpt;
    float yawIn, yawOut, yawSetpt;
    float motorTL, motorTR, motorBL, motorBR;
    float velX, velY, velZ;
};

static const int LOG_BUFFER_SIZE = 1000;
LogEntry logBuffer[LOG_BUFFER_SIZE];
int logCount = 0;

WebServer webServer(80);
bool webServerStarted = false;

// IMU
static const uint8_t BNO08X_SCK = 7;
static const uint8_t BNO08X_MISO = 15;
static const uint8_t BNO08X_MOSI = 5;
static const uint8_t BNO08X_CS = 4;
static const uint8_t BNO08X_INT = 16;
static const int8_t BNO08X_RST = 6;

Adafruit_BNO08x bno085(BNO08X_RST);

double yaw, pitch, roll;
double gyroPitch = 0.0; // deg/s — pitch angular rate for depth control
double velX = 0.0, velY = 0.0, velZ = 0.0;
unsigned long lastAccelTime = 0;
sh2_SensorValue_t sensorValue;

// Timing
static const double pathTime = 6000;
unsigned long timer = 0;

// PIDs
static double baseSpeed = 0.7;
static double stabilizeSpeed = 0.15;

// Yaw PID
double yawInput, yawOutput, yawSetpoint;

double yawkp = 0.08;
double yawki = 0.0;
double yawkd = 0.0;
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, yawkp, yawki, yawkd, DIRECT);

// Pitch PD — P on angle error (targets level), D on gyro rate (fast response)
double pitchOutput, pitchSetpoint = 0.0;
double pitchkp = 0.03;
double pitchkd = 0.001;

// Roll PID
double rollInput, rollOutput, rollSetpoint;
double rollkp = 0.01;
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

void chime(int count, int delayMs) {
    for (int i = 0; i < count; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(delayMs);
        digitalWrite(BUZZER_PIN, LOW);
        delay(delayMs);
    }
}

void serveLogs() {
    webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    webServer.send(200, "text/csv", "");
    webServer.sendContent(
        "timestamp_ms,yaw,pitch,roll,"
        "pitchIn,pitchOut,pitchSetpt,"
        "rollIn,rollOut,rollSetpt,"
        "yawIn,yawOut,yawSetpt,"
        "motorTL,motorTR,motorBL,motorBR,"
        "velX,velY,velZ\n"
    );
    char line[256];
    for (int i = 0; i < logCount; i++) {
        const LogEntry& e = logBuffer[i];
        snprintf(line, sizeof(line),
            "%lu,%.3f,%.3f,%.3f,"
            "%.4f,%.4f,%.4f,"
            "%.4f,%.4f,%.4f,"
            "%.4f,%.4f,%.4f,"
            "%.4f,%.4f,%.4f,%.4f,"
            "%.4f,%.4f,%.4f\n",
            (unsigned long)e.timestamp,
            e.yaw, e.pitch, e.roll,
            e.pitchIn, e.pitchOut, e.pitchSetpt,
            e.rollIn, e.rollOut, e.rollSetpt,
            e.yawIn, e.yawOut, e.yawSetpt,
            e.motorTL, e.motorTR, e.motorBL, e.motorBR,
            e.velX, e.velY, e.velZ);
        webServer.sendContent(line);
    }
    webServer.sendContent("");
    chime(3, 100);
    Serial.printf("Served %d log entries\n", logCount);
}

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

    if (!webServerStarted) {
        webServer.on("/logs", HTTP_GET, serveLogs);
        webServerStarted = true;
    }
    webServer.begin();
    Serial.printf("Log server up at http://%s/logs (%d entries)\n", WiFi.localIP().toString().c_str(), logCount);
    chime(2, 200);
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
        if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
            float qw = sensorValue.un.rotationVector.real;
            float qx = sensorValue.un.rotationVector.i;
            float qy = sensorValue.un.rotationVector.j;
            float qz = sensorValue.un.rotationVector.k;

            yaw   = toDegrees(atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)));
            pitch = toDegrees(asin(2.0 * (qw * qy - qz * qx)));
            roll  = toDegrees(atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)));
            roll -= 180.0;
            if (roll < -180.0) roll += 360.0;
        } else if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
            // Y axis = pitch rate — swap to gyroscope.x if IMU is mounted 90° rotated
            gyroPitch = toDegrees(sensorValue.un.gyroscope.y);
        } else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
            unsigned long now = millis();
            if (lastAccelTime > 0) {
                float dt = (now - lastAccelTime) / 1000.0f;
                velX += sensorValue.un.linearAcceleration.x * dt;
                velY += sensorValue.un.linearAcceleration.y * dt;
                velZ += sensorValue.un.linearAcceleration.z * dt;
            }
            lastAccelTime = now;
        }
    }
}

void updatePID() {
    yawPID.Compute();
    pitchOutput = pitchkp * (pitchSetpoint - pitch) - pitchkd * gyroPitch;
    pitchOutput = constrain(pitchOutput, -0.5, 0.5);
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

void stabilize() {
    updateIMU();

    pitchOutput = constrain(pitchkp * (pitchSetpoint - pitch) - pitchkd * gyroPitch, -stabilizeSpeed, stabilizeSpeed);
    rollInput = roll;
    rollPID.Compute();

    float tl = stabilizeSpeed - pitchOutput - rollOutput;
    float tr = stabilizeSpeed - pitchOutput + rollOutput;
    float bl = stabilizeSpeed + pitchOutput + rollOutput;
    float br = stabilizeSpeed + pitchOutput - rollOutput;

    topLeftMotor.setSpeed(tl);
    topRightMotor.setSpeed(tr);
    bottomLeftMotor.setSpeed(bl);
    bottomRightMotor.setSpeed(br);
}

void path() {
    updateIMU();

    if (yaw < 0) yaw += 360.0;

    double yawError = -calculateError(yaw, heading);

    yawInput = -yawError;
    rollInput = roll;

    updatePID();

    float tl = baseSpeed - pitchOutput + yawOutput - rollOutput;
    float tr = baseSpeed - pitchOutput - yawOutput + rollOutput;
    float bl = baseSpeed + pitchOutput + yawOutput + rollOutput;
    float br = baseSpeed + pitchOutput - yawOutput - rollOutput;

    bottomLeftMotor.setSpeed(bl);
    bottomRightMotor.setSpeed(br);
    topLeftMotor.setSpeed(tl);
    topRightMotor.setSpeed(tr);

    if (logCount < LOG_BUFFER_SIZE) {
        LogEntry& e = logBuffer[logCount++];
        e.timestamp  = millis();
        e.yaw        = (float)yaw;         e.pitch      = (float)pitch;       e.roll       = (float)roll;
        e.pitchIn    = (float)pitch;        e.pitchOut   = (float)pitchOutput; e.pitchSetpt = (float)pitchSetpoint;
        e.rollIn     = (float)rollInput;   e.rollOut    = (float)rollOutput;  e.rollSetpt  = (float)rollSetpoint;
        e.yawIn      = (float)yawInput;    e.yawOut     = (float)yawOutput;   e.yawSetpt   = (float)yawSetpoint;
        e.motorTL = tl; e.motorTR = tr; e.motorBL = bl; e.motorBR = br;
        e.velX = (float)velX; e.velY = (float)velY; e.velZ = (float)velZ;
    }
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
    bno085.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
    bno085.enableReport(SH2_LINEAR_ACCELERATION, 10000);

    // PID Setup
    rollPID.SetMode(AUTOMATIC);
    yawPID.SetMode(AUTOMATIC);

    rollPID.SetOutputLimits(-0.5, 0.5);
    yawPID.SetOutputLimits(-0.5, 0.5);

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
    if (wifiConnected) webServer.handleClient();

    switch (runState) {

        case RunState::Idle:
            if (!wifiConnected) connectToWiFi();
            if (reedSwitch.isPressed()) {
                Serial.println(">>> Transitioning Idle -> Armed");
                WiFi.disconnect();
                wifiConnected = false;
                runState = RunState::Armed;
            }
            break;

        case RunState::Armed:
            stabilize();
            armedBeep();
            if (reedSwitch.released()) {
                if (yaw < 0) yaw += 360.0;
                heading = yaw;
                logCount = 0;
                velX = velY = velZ = 0.0;
                lastAccelTime = 0;
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