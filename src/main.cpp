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

// Battery voltage — 47k/10k divider on ADC1 pin
static const uint8_t VBAT_PIN = 2;
static const float VDIV_RATIO = 5.7f;   // (47k + 10k) / 10k
static const float VBAT_CAL   = 1.021f; // calibration factor for ADC/resistor tolerance

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
    uint8_t state;  // 0=Armed, 1=Running, 2=Recovery
    float voltage;
    float yaw, pitch, roll;
    float pitchIn, pitchOut, pitchSetpt;
    float rollIn, rollOut, rollSetpt;
    float yawIn, yawOut, yawSetpt;
    float motorTL, motorTR, motorBL, motorBR;
};

// 600 entries: room for armed stabilization + 5s run + recovery
static const int LOG_BUFFER_SIZE = 600;
LogEntry logBuffer[LOG_BUFFER_SIZE];
int logCount = 0;
int runCount = 0;              // increments each Armed->Running transition (per power cycle)
unsigned long runEndTime = 0;  // set when run finishes for duration calc

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
static const double PICKUP_PITCH_DEG  = 45.0;  // pitch threshold for pickup detection
static const double RECOVERY_DEGREES  = 50.0;  // degrees left to turn in recovery
static const double RECOVERY_TIME_MS  = 5000;  // max recovery duration before stopping
static const unsigned long RAMP_UP_MS = 200;   // acceleration ramp from stabilize to full speed
static const double MID_TURN_DEG     = 25.0;  // degrees to turn mid-run
static const unsigned long MID_TURN_MS = 700; // ms into run to start mid-turn
static const double pathTime = 5500; // 4500 m for perfection poggies omg nice one lol
static const double MAX_RIGHT_DEV_DEG = 30.0;  // abort if sub drifts this far right of heading
static const double MAX_RIGHT_YAW_OUT = 0.15;  // cap rightward yaw correction (sub only goes straight/left)
unsigned long timer = 0;
unsigned long runStart = 0;
unsigned long recoveryTimer = 0;
double recoveryHeading = 0.0;

// PIDs
static double stabilizeSpeed = 0.10;

// Yaw PID
double yawInput, yawOutput, yawSetpoint;

double yawkp = 0.02;
double yawki = 0.0;
double yawkd = 0.0;
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, yawkp, yawki, yawkd, DIRECT);

// Pitch PD — P on angle error (targets level), D on gyro rate (fast response)
static const double BASE_PITCH_DEG    = 4;  // level-flight pitch setpoint
double pitchOutput, pitchSetpoint = BASE_PITCH_DEG;
double pitchkp = 0.04;
double pitchkd = 0.001;

// Roll PID
double rollInput, rollOutput, rollSetpoint;
double rollkp = 0.03;
double rollki = 0.0;
double rollkd = 0.0;
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, rollkp, rollki, rollkd, DIRECT);

enum class RunState {
    Idle,
    Armed,
    Running,
    Recovery,
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
        "run_number,timestamp_ms,state,voltage,"
        "yaw,pitch,roll,"
        "pitchIn,pitchOut,pitchSetpt,"
        "rollIn,rollOut,rollSetpt,"
        "yawIn,yawOut,yawSetpt,"
        "motorTL,motorTR,motorBL,motorBR\n"
    );
    char line[300];
    for (int i = 0; i < logCount; i++) {
        const LogEntry& e = logBuffer[i];
        snprintf(line, sizeof(line),
            "%d,%lu,%u,%.2f,"
            "%.3f,%.3f,%.3f,"
            "%.4f,%.4f,%.4f,"
            "%.4f,%.4f,%.4f,"
            "%.4f,%.4f,%.4f,"
            "%.4f,%.4f,%.4f,%.4f\n",
            runCount,
            (unsigned long)e.timestamp,
            (unsigned)e.state, e.voltage,
            e.yaw, e.pitch, e.roll,
            e.pitchIn, e.pitchOut, e.pitchSetpt,
            e.rollIn, e.rollOut, e.rollSetpt,
            e.yawIn, e.yawOut, e.yawSetpt,
            e.motorTL, e.motorTR, e.motorBL, e.motorBR);
        webServer.sendContent(line);
    }
    webServer.sendContent("");

    // Print run duration
    if (runEndTime > 0 && runStart > 0) {
        Serial.printf("Run duration: %.2f s\n", (runEndTime - runStart) / 1000.0f);
    }
    chime(3, 100);
    Serial.printf("Served %d log entries\n", logCount);
}

// WiFi — non-blocking connect with timeout
bool wifiConnecting = false;
unsigned long wifiConnectStart = 0;
static const unsigned long WIFI_TIMEOUT_MS = 10000;

void startWiFiConnect() {
    if (wifiConnecting) return;
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    wifiConnecting = true;
    wifiConnectStart = millis();
    Serial.println("\nConnecting to WiFi Network ..");
}

// Call each loop iteration — returns true once connected
bool tryWiFiConnect() {
    if (wifiConnected) return true;
    if (!wifiConnecting) { startWiFiConnect(); return false; }

    if (WiFi.status() == WL_CONNECTED) {
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
        wifiConnecting = false;
        return true;
    }

    if (millis() - wifiConnectStart > WIFI_TIMEOUT_MS) {
        Serial.println("\nWiFi connection timed out");
        WiFi.disconnect();
        wifiConnecting = false;
        return false;
    }
    return false;
}

float toDegrees(double radians) {
    return radians * 180.0 / M_PI;
}

float readBatteryVoltage() {
    int raw = analogRead(VBAT_PIN);
    return (raw * 3.3f / 4095.0f) * VDIV_RATIO * VBAT_CAL;
}

void stopMotors() {
    bottomLeftMotor.stop();
    bottomRightMotor.stop();
    topLeftMotor.stop();
    topRightMotor.stop();
}


// Returns true when a fresh rotation vector was received (use to gate logging)
bool updateIMU() {
    if (!bno085.getSensorEvent(&sensorValue)) return false;

    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
        float qw = sensorValue.un.gameRotationVector.real;
        float qx = sensorValue.un.gameRotationVector.i;
        float qy = sensorValue.un.gameRotationVector.j;
        float qz = sensorValue.un.gameRotationVector.k;

        yaw   = toDegrees(atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)));
        pitch = toDegrees(asin(2.0 * (qw * qy - qz * qx)));
        roll  = toDegrees(atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)));
        roll -= 180.0;
        if (roll < -180.0) roll += 360.0;
        return true;
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
    return false;
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

void logEntry(uint8_t state, float tl, float tr, float bl, float br) {
    if (logCount >= LOG_BUFFER_SIZE) return;
    LogEntry& e = logBuffer[logCount++];
    e.timestamp  = millis();
    e.state      = state;
    e.voltage    = readBatteryVoltage();
    e.yaw        = (float)yaw;         e.pitch      = (float)pitch;       e.roll       = (float)roll;
    e.pitchIn    = (float)pitch;        e.pitchOut   = (float)pitchOutput; e.pitchSetpt = (float)pitchSetpoint;
    e.rollIn     = (float)rollInput;   e.rollOut    = (float)rollOutput;  e.rollSetpt  = (float)rollSetpoint;
    e.yawIn      = (float)yawInput;    e.yawOut     = (float)yawOutput;   e.yawSetpt   = (float)yawSetpoint;
    e.motorTL = tl; e.motorTR = tr; e.motorBL = bl; e.motorBR = br;
}

void stabilize() {
    bool newRotation = updateIMU();

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

    if (newRotation) logEntry(0, tl, tr, bl, br);
}

void runMotors(double targetHeading, float maxSpeed = 1.0f, uint8_t logState = 1) {
    bool newRotation = updateIMU();

    if (yaw < 0) yaw += 360.0;

    pitchSetpoint = BASE_PITCH_DEG;

    double yawError = -calculateError(yaw, targetHeading);

    yawInput = -yawError;
    rollInput = roll;

    updatePID();

    // Cap rightward yaw correction — sub only goes straight or left
    if (yawOutput > MAX_RIGHT_YAW_OUT) yawOutput = MAX_RIGHT_YAW_OUT;

    // Raw correction per motor (no base — will be normalised below)
    float c_tl = -pitchOutput + yawOutput - rollOutput;
    float c_tr = -pitchOutput - yawOutput + rollOutput;
    float c_bl =  pitchOutput + yawOutput + rollOutput;
    float c_br =  pitchOutput - yawOutput - rollOutput;

    // Normalise into [0, 1] — fastest motor at 1.0, preserve all differentials
    float maxC = max(max(c_tl, c_tr), max(c_bl, c_br));
    float minC = min(min(c_tl, c_tr), min(c_bl, c_br));
    float range = maxC - minC;

    float tl, tr, bl, br;
    if (range > 1.0f) {
        // Corrections too wide — scale to preserve ratios
        float s = 1.0f / range;
        tl = (c_tl - minC) * s;
        tr = (c_tr - minC) * s;
        bl = (c_bl - minC) * s;
        br = (c_br - minC) * s;
    } else {
        // Shift so fastest motor = 1.0, all others >= 0
        tl = 1.0f - (maxC - c_tl);
        tr = 1.0f - (maxC - c_tr);
        bl = 1.0f - (maxC - c_bl);
        br = 1.0f - (maxC - c_br);
    }

    tl *= maxSpeed;
    tr *= maxSpeed;
    bl *= maxSpeed;
    br *= maxSpeed;

    bottomLeftMotor.setSpeed(bl);
    bottomRightMotor.setSpeed(br);
    topLeftMotor.setSpeed(tl);
    topRightMotor.setSpeed(tr);

    if (newRotation) logEntry(logState, tl, tr, bl, br);
}

void setup() {
    bottomLeftMotor.begin();
    bottomRightMotor.begin();
    topLeftMotor.begin();
    topRightMotor.begin();

    delay(1000);
    Serial.begin(115200);
    startWiFiConnect();

    // IMU Setup
    SPI.begin(BNO08X_SCK, BNO08X_MISO, BNO08X_MOSI, BNO08X_CS);
    if (!bno085.begin_SPI(BNO08X_CS, BNO08X_INT)) {
        Serial.println("Failed to initialize BNO08x!");
    }

    bno085.enableReport(SH2_GAME_ROTATION_VECTOR, 20000);  // no magnetometer — immune to motor interference
    bno085.enableReport(SH2_GYROSCOPE_CALIBRATED, 20000);
    bno085.enableReport(SH2_LINEAR_ACCELERATION, 20000);

    // PID Setup
    rollPID.SetMode(AUTOMATIC);
    yawPID.SetMode(AUTOMATIC);

    rollPID.SetOutputLimits(-0.5, 0.5);
    rollPID.SetSampleTime(20);
    yawPID.SetOutputLimits(-0.5, 0.5);
    yawPID.SetSampleTime(20);

    //Pin configurations
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    analogReadResolution(12);
    pinMode(VBAT_PIN, INPUT);

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
            tryWiFiConnect();
            if (reedSwitch.isPressed()) {
                Serial.println(">>> Transitioning Idle -> Armed");
                WiFi.disconnect();
                wifiConnected = false;
                wifiConnecting = false;
                logCount = 0;
                runEndTime = 0;
                yawInput = 0; yawOutput = 0;
                runState = RunState::Armed;
            }
            break;

        case RunState::Armed:
            stabilize();
            armedBeep();
            if (reedSwitch.released()) {
                if (yaw < 0) yaw += 360.0;
                heading = yaw;
                velX = velY = velZ = 0.0;
                lastAccelTime = 0;

                // Reset PID state so integral doesn't carry over from previous run
                yawOutput = 0; yawInput = 0;
                yawPID.SetMode(MANUAL);
                yawPID.SetMode(AUTOMATIC);
                rollOutput = 0; rollInput = 0;
                rollPID.SetMode(MANUAL);
                rollPID.SetMode(AUTOMATIC);

                runCount++;
                Serial.printf(">>> Transitioning Armed -> Running | Run #%d | Captured heading: %.1f\n", runCount, heading);
                digitalWrite(BUZZER_PIN, LOW);
                runStart = millis();
                timer = runStart + (unsigned long)pathTime;
                runState = RunState::Running;
            }
            break;

        case RunState::Running:
            if (fabs(pitch) > PICKUP_PITCH_DEG) {
                Serial.println(">>> Pickup detected during run!");
                runState = RunState::Finished;
            } else if (millis() <= timer) {
                unsigned long elapsed = millis() - runStart;
                float ramp = (elapsed < RAMP_UP_MS)
                    ? stabilizeSpeed + (1.0f - stabilizeSpeed) * ((float)elapsed / RAMP_UP_MS)
                    : 1.0f;
                double targetHeading = (elapsed >= MID_TURN_MS)
                    ? fmod(heading + MID_TURN_DEG, 360.0)
                    : heading;
                runMotors(targetHeading, ramp);
                // Safeguard: abort if sub drifts too far right of original heading
                double rightDev = calculateError(yaw, heading);
                if (rightDev > MAX_RIGHT_DEV_DEG) {
                    Serial.printf(">>> ABORT: %.1f° right of heading (limit %.1f°)\n",
                                  rightDev, MAX_RIGHT_DEV_DEG);
                    stopMotors();
                    runState = RunState::Finished;
                }
            } else {
                recoveryHeading = fmod(heading + RECOVERY_DEGREES, 360.0);
                recoveryTimer = millis() + (unsigned long)RECOVERY_TIME_MS;
                Serial.printf(">>> Transitioning Running -> Recovery | Recovery heading: %.1f\n", recoveryHeading);
                runState = RunState::Recovery;
            }
            break;

        case RunState::Recovery:
            if (fabs(pitch) > PICKUP_PITCH_DEG) {
                Serial.println(">>> Pickup detected during recovery!");
                runState = RunState::Finished;
            } else if (millis() > recoveryTimer) {
                Serial.println(">>> Recovery timeout, stopping");
                runState = RunState::Finished;
            } else {
                runMotors(recoveryHeading, 1.0f, 2);
                double rightDev = calculateError(yaw, heading);
                if (rightDev > MAX_RIGHT_DEV_DEG) {
                    Serial.printf(">>> ABORT (recovery): %.1f° right of heading\n", rightDev);
                    stopMotors();
                    runState = RunState::Finished;
                }
            }
            break;

        case RunState::Finished:
            stopMotors();
            // Backtrack through log to find last sample where pitch was still flat —
            // more accurate than millis() which fires after the 45° threshold is crossed.
            runEndTime = millis();
            for (int i = logCount - 1; i >= 0; i--) {
                if (fabs(logBuffer[i].pitch) < PICKUP_PITCH_DEG / 2.0f) {
                    runEndTime = logBuffer[i].timestamp;
                    break;
                }
            }
            if (runStart > 0) {
                Serial.printf(">>> Run duration: %.2f s\n", (runEndTime - runStart) / 1000.0f);
            }
            Serial.println(">>> Transitioning Finished -> Idle");
            runState = RunState::Idle;
            break;
    }
}