#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <SPI.h>
#include <Adafruit_BNO08x.h>
#include "Motor.h"

// WiFi credentials (fill these before flashing)
static const char* WIFI_SSID = "Suboptimal";
static const char* WIFI_PASS = "Suboptimal123";

// Reed switch: magnet held = closed/LOW, release = start mission
static const uint8_t REED_SWITCH_PIN = 4;

// BNO08x pins for ESP32-S3 SPI
static const uint8_t BNO08X_SCK = 18;
static const uint8_t BNO08X_MISO = 17;
static const uint8_t BNO08X_MOSI = 16;
static const uint8_t BNO08X_CS = 15;
static const uint8_t BNO08X_INT = 14;
static const int8_t BNO08X_RST = 13;

// ESC timing
static const int PWM_FREQ = 50;
static const int PWM_RES_BITS = 10;
static const bool ENABLE_ESC_CALIBRATION = false; // set true only when intentionally calibrating
static const bool ENABLE_MOTOR_FEEDBACK_BEEPS = true; // disable motor pulse beeps to avoid ESC tone/confusion
static const bool ENABLE_ESC_HARD_SIGNAL_CUT = false; // keep false for these ESCs; hard cut can trigger calibration/lost-signal states
static const uint16_t ESC_CALIBRATE_HIGH_MS = 5000;
static const uint16_t ESC_CALIBRATE_LOW_MS = 8000;
static const uint16_t ESC_HARD_CUT_HOLD_MS = 250;

// Motors
Motor bottomLeftMotor(9, 0, PWM_FREQ, PWM_RES_BITS);
Motor bottomRightMotor(10, 1, PWM_FREQ, PWM_RES_BITS);
Motor topLeftMotor(11, 2, PWM_FREQ, PWM_RES_BITS);
Motor topRightMotor(12, 3, PWM_FREQ, PWM_RES_BITS);

// Mission tuning
static const float BASE_THROTTLE = 0.45f;
static const float CURVE_BIAS = 0.05f;
static const uint32_t MISSION_DURATION_MS = 5000;

// Compass alignment target
static const float TARGET_HEADING_DEG = 53.0f;
static const float ALIGN_TOL_DEG = 5.0f;
static const float CHIRP_SPEED = 0.08f;
static const uint16_t CHIRP_MS = 240;
static const uint16_t REED_DEBOUNCE_MS = 120;
static const uint16_t ESC_DISARM_MS = 2500;

// Euler remap (for sensor mounting orientation)
// Native indices: 0=roll, 1=pitch, 2=yaw
// Change these if your physical mount swaps axes.
static const uint8_t ROLL_FROM_IDX = 2;
static const int8_t ROLL_SIGN = 1;
static const uint8_t PITCH_FROM_IDX = 1;
static const int8_t PITCH_SIGN = 1;
static const uint8_t YAW_FROM_IDX = 0;
static const int8_t YAW_SIGN = 1;

// IMU
Adafruit_BNO08x bno08x(BNO08X_RST);
sh2_SensorValue_t sensorValue;
bool imuReady = false;
float rollDeg = 0.0f;
float pitchDeg = 0.0f;
float yawDeg = 0.0f;

enum class RunState {
    WaitingForMagnet,
    Armed,
    RunningMission,
    Finished
};

RunState runState = RunState::WaitingForMagnet;
uint32_t missionStartMs = 0;
uint32_t lastDebugMs = 0;
uint32_t lastGuideBeepMs = 0;
bool finalAlignBeepDone = false;
float lastHeadingErrDeg = 0.0f;
bool finishedHardStopped = false;
bool wifiStarted = false;
uint32_t lastWifiRetryMs = 0;
bool otaStarted = false;
bool ipPrinted = false;
bool reedRawClosed = false;
bool reedDebouncedClosed = false;
uint32_t reedLastChangeMs = 0;

const char* runStateToString(RunState s) {
    switch (s) {
        case RunState::WaitingForMagnet: return "WaitingForMagnet";
        case RunState::Armed: return "Armed";
        case RunState::RunningMission: return "RunningMission";
        case RunState::Finished: return "Finished";
        default: return "Unknown";
    }
}

void stopAllMotors() {
    bottomLeftMotor.stop();
    bottomRightMotor.stop();
    topLeftMotor.stop();
    topRightMotor.stop();
}

void hardStopAllMotors() {
    bottomLeftMotor.hardStopSignal();
    bottomRightMotor.hardStopSignal();
    topLeftMotor.hardStopSignal();
    topRightMotor.hardStopSignal();
}

void disarmEscStartup() {
    Serial.println("[ESC] Disarm sequence...");
    stopAllMotors();
    delay(ESC_DISARM_MS);
    // Keep valid 1000us pulses continuously so ESCs do not enter calibration/lost mode.
    stopAllMotors();
    Serial.println("[ESC] Disarm done");
}

void calibrateEscIfEnabled() {
    if (!ENABLE_ESC_CALIBRATION) return;

    Serial.println("[ESC] Calibration enabled: HIGH then LOW throttle");
    Serial.println("[ESC] Keep prop area clear");

    // High throttle calibration stage
    bottomLeftMotor.setSpeed(1.0f);
    bottomRightMotor.setSpeed(1.0f);
    topLeftMotor.setSpeed(1.0f);
    topRightMotor.setSpeed(1.0f);
    delay(ESC_CALIBRATE_HIGH_MS);

    // Low throttle calibration stage
    stopAllMotors();
    delay(ESC_CALIBRATE_LOW_MS);

    hardStopAllMotors();
    delay(120);
    Serial.println("[ESC] Calibration sequence complete");
}

float shortestHeadingErrDeg(float target, float current) {
    float err = target - current;
    while (err > 180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

void motorBeepPulse(uint16_t onMs = CHIRP_MS) {
    if (!ENABLE_MOTOR_FEEDBACK_BEEPS) return;
    bottomLeftMotor.setSpeed(CHIRP_SPEED);
    bottomRightMotor.setSpeed(CHIRP_SPEED);
    topLeftMotor.setSpeed(CHIRP_SPEED);
    topRightMotor.setSpeed(CHIRP_SPEED);
    delay(onMs);
    stopAllMotors();
}

void motorBeepPattern(uint8_t count, uint16_t onMs = CHIRP_MS, uint16_t gapMs = 90) {
    if (!ENABLE_MOTOR_FEEDBACK_BEEPS) return;
    // NOTE: true ESC power-up beeps are firmware-generated by ESC and usually
    // cannot be commanded directly. This is a pulse-based feedback pattern.
    for (uint8_t i = 0; i < count; i++) {
        motorBeepPulse(onMs);
        if (i + 1 < count) delay(gapMs);
    }
}

uint32_t guidanceBeepIntervalMs(float absErrDeg) {
    // Closer to target => faster beeps. Farther => slower beeps.
    float e = absErrDeg;
    if (e < 0.0f) e = 0.0f;
    if (e > 60.0f) e = 60.0f;
    return (uint32_t)(350.0f + (e / 60.0f) * (1400.0f - 350.0f));
}

void updateReedDebounce() {
    const bool rawClosedNow = digitalRead(REED_SWITCH_PIN) == LOW;
    const uint32_t now = millis();

    if (rawClosedNow != reedRawClosed) {
        reedRawClosed = rawClosedNow;
        reedLastChangeMs = now;
    }

    if ((now - reedLastChangeMs) >= REED_DEBOUNCE_MS && reedDebouncedClosed != reedRawClosed) {
        reedDebouncedClosed = reedRawClosed;
        Serial.print("[REED] Debounced: ");
        Serial.println(reedDebouncedClosed ? "CLOSED (magnet present)" : "OPEN (magnet removed)");
    }
}

void setCurveThrottle() {
    const float leftCmd = max(0.0f, BASE_THROTTLE - CURVE_BIAS);
    const float rightCmd = min(1.0f, BASE_THROTTLE + CURVE_BIAS);
    bottomLeftMotor.setSpeed(leftCmd);
    topLeftMotor.setSpeed(leftCmd);
    bottomRightMotor.setSpeed(rightCmd);
    topRightMotor.setSpeed(rightCmd);
}

void connectWifiIfNeeded() {
    if (strlen(WIFI_SSID) == 0) return;

    const uint32_t now = millis();

    if (!wifiStarted) {
        WiFi.persistent(false);
        WiFi.setSleep(false);
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        wifiStarted = true;
        lastWifiRetryMs = now;
        return;
    }

    if (WiFi.status() != WL_CONNECTED && now - lastWifiRetryMs > 5000) {
        WiFi.reconnect();
        lastWifiRetryMs = now;
    }
}

void startOtaIfNeeded() {
    if (otaStarted) return;
    if (WiFi.status() != WL_CONNECTED) return;
    ArduinoOTA.setHostname("submarine");
    ArduinoOTA.onStart([]() {
        stopAllMotors();
    });
    ArduinoOTA.begin();
    otaStarted = true;
}

void printIpIfConnected() {
    if (ipPrinted) return;
    if (WiFi.status() != WL_CONNECTED) return;
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
    ipPrinted = true;
}

void readImu() {
    if (!imuReady) return;
    
    if (bno08x.wasReset()) {
        bno08x.enableReport(SH2_ARVR_STABILIZED_RV);
    }
    
    if (!bno08x.getSensorEvent(&sensorValue)) return;

    if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
        const float qr = sensorValue.un.gameRotationVector.real;
        const float qi = sensorValue.un.gameRotationVector.i;
        const float qj = sensorValue.un.gameRotationVector.j;
        const float qk = sensorValue.un.gameRotationVector.k;

        const float sqr = sq(qr);
        const float sqi = sq(qi);
        const float sqj = sq(qj);
        const float sqk = sq(qk);

        const float nativeRollDeg = atan2f(2.0f * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr)) * RAD_TO_DEG;
        const float nativePitchDeg = asinf(-2.0f * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr)) * RAD_TO_DEG;
        const float nativeYawDeg = atan2f(2.0f * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr)) * RAD_TO_DEG;

        const float nativeEuler[3] = {nativeRollDeg, nativePitchDeg, nativeYawDeg};
        rollDeg = nativeEuler[ROLL_FROM_IDX] * (float)ROLL_SIGN;
        pitchDeg = nativeEuler[PITCH_FROM_IDX] * (float)PITCH_SIGN;
        yawDeg = nativeEuler[YAW_FROM_IDX] * (float)YAW_SIGN;

        if (yawDeg < 0.0f) yawDeg += 360.0f;
    }
}

void updateArmState() {
    static bool lastClosed = false;
    static RunState lastPrintedState = RunState::WaitingForMagnet;
    const bool closed = reedDebouncedClosed;

    if (closed) {
        finishedHardStopped = false;
        // Entering armed mode
        if (!lastClosed) {
            Serial.println("[ARM] Magnet detected -> armed");
            motorBeepPattern(1, 220, 80);  // initial armed beep
            lastGuideBeepMs = 0;
            finalAlignBeepDone = false;
        }

        // Allow re-arming even after a completed run (no power cycle required).
        runState = RunState::Armed;
        stopAllMotors();

        if (imuReady) {
            const float err = shortestHeadingErrDeg(TARGET_HEADING_DEG, yawDeg);
            const float absErr = fabsf(err);
            lastHeadingErrDeg = err;

            if (absErr <= ALIGN_TOL_DEG) {
                if (!finalAlignBeepDone) {
                    Serial.println("[ALIGN] On target -> pause then double beep");
                    delay(220);
                    motorBeepPattern(2, 220, 90);
                    finalAlignBeepDone = true;
                }
            } else {
                const uint32_t now = millis();
                const uint32_t intervalMs = guidanceBeepIntervalMs(absErr);
                if (lastGuideBeepMs == 0 || now - lastGuideBeepMs >= intervalMs) {
                    motorBeepPulse(112);  // guidance tick
                    lastGuideBeepMs = now;
                }
            }
        }
    } else if (lastClosed && runState == RunState::Armed) {
        runState = RunState::RunningMission;
        missionStartMs = millis();
        Serial.println("[MISSION] Magnet removed while armed -> mission started");
    }

    if (runState != lastPrintedState) {
        Serial.print("[STATE] ");
        Serial.println(runStateToString(runState));
        lastPrintedState = runState;
    }

    lastClosed = closed;
}

void updateMission() {
    if (runState == RunState::RunningMission) {
        finishedHardStopped = false;
        setCurveThrottle();
        if (millis() - missionStartMs >= MISSION_DURATION_MS) {
            stopAllMotors();
            runState = RunState::Finished;
            Serial.println("[MISSION] Completed 4s run -> stopped");
        }
    } else if (runState == RunState::Finished) {
        // Hold minimum valid throttle continuously.
        stopAllMotors();
        finishedHardStopped = false;
    }
}

void printDebugHeartbeat() {
    const uint32_t now = millis();
    if (now - lastDebugMs < 500) return;
    lastDebugMs = now;

    Serial.print("[DBG] reedRaw=");
    Serial.print(reedRawClosed ? "CLOSED" : "OPEN");
    Serial.print(" reed=");
    Serial.print(reedDebouncedClosed ? "CLOSED" : "OPEN");
    Serial.print(" state=");
    Serial.print(runStateToString(runState));
    Serial.print(" imu=");
    Serial.print(imuReady ? "OK" : "NO");
    Serial.print(" roll=");
    Serial.print(rollDeg, 1);
    Serial.print(" pitch=");
    Serial.print(pitchDeg, 1);
    Serial.print(" yaw=");
    Serial.print(yawDeg, 1);
    Serial.print(" err=");
    Serial.print(lastHeadingErrDeg, 1);
    Serial.print(" wifi=");
    Serial.println(WiFi.status() == WL_CONNECTED ? "UP" : "DOWN");
}

void setupMotors() {
    bottomLeftMotor.begin();
    bottomRightMotor.begin();
    topLeftMotor.begin();
    topRightMotor.begin();
    stopAllMotors();
    disarmEscStartup();
}

void setup() {
    Serial.begin(115200);
    Serial.println("Booting...");
    pinMode(REED_SWITCH_PIN, INPUT_PULLUP);
    Serial.println("[REED] INPUT_PULLUP enabled; CLOSED should read LOW");

    Serial.println("Setting Up Motors...");
    setupMotors();

    Serial.println("Setting Up IMU...");
    SPI.begin(BNO08X_SCK, BNO08X_MISO, BNO08X_MOSI, BNO08X_CS);
    if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT, &SPI)) {
        Serial.println("Failed to find BNO08x chip");
    } else {
        Serial.println("BNO08x Found!");
        bno08x.enableReport(SH2_ARVR_STABILIZED_RV);
        imuReady = true;
    }

    Serial.println("Connecting to Wifi...");
    connectWifiIfNeeded();

    Serial.println("Starting OTA...");
    startOtaIfNeeded();
    Serial.println("Setup done");

    calibrateEscIfEnabled();

}

void loop() {
    connectWifiIfNeeded();
    startOtaIfNeeded();
    printIpIfConnected();
    updateReedDebounce();
    
    if (WiFi.status() == WL_CONNECTED) {
        ArduinoOTA.handle();
    }
    
    readImu();
    updateArmState();
    updateMission();
    printDebugHeartbeat();
    
    delay(10);
}