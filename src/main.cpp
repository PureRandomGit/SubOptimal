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
static const uint8_t REED_SWITCH_PIN = 18;
Bounce2::Button reedSwitch = Bounce2::Button();

// Buzzer Pin
static const uint8_t BUZZER_PIN = 8;
static const double heading = 30.0; // Desired heading in degrees

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
static const double pathTime = 5000; // 5 seconds
unsigned long timer = 0;

// PIDs

static double baseSpeed = 0.05;

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

void headingBeep() {
    updateIMU();

    if (yaw < 0) yaw += 360.0;

    double diff = abs(yaw - heading);
    if (diff > 180.0) diff = 360.0 - diff;  // shortest angular distance

    // Faster beeps = closer to target heading
    unsigned long interval = (unsigned long)map((long)diff, 0, 180, 100, 1500);

    static unsigned long lastToggle = 0;
    static bool buzzerOn = false;
    unsigned long now = millis();

    if (now - lastToggle >= (buzzerOn ? 50UL : interval)) {
        buzzerOn = !buzzerOn;
        digitalWrite(BUZZER_PIN, buzzerOn ? HIGH : LOW);
        lastToggle = now;
    }
    if (diff < 2.0) {
        // Solid tone when within 5 degrees
        digitalWrite(BUZZER_PIN, HIGH);
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
    // Motors first — pins must output min throttle (1000us) before ESCs finish booting
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

    rollPID.SetOutputLimits(-0.1, 0.1);
    pitchPID.SetOutputLimits(-0.1, 0.1);
    yawPID.SetOutputLimits(-0.1, 0.1);

    //Pin configurations
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    reedSwitch.attach(REED_SWITCH_PIN, INPUT_PULLUP);
    reedSwitch.interval(500);
    reedSwitch.setPressedState(false); // TODO: Test if this is the correct state

    Serial.println("Setup complete, entering main loop.");

}

void loop() {

    reedSwitch.update();

    ArduinoOTA.handle();  // Handles OTA

    unsigned long now = millis();

    // Capture edge events once — Bounce2 consumes these on first call
    bool reedPressed  = reedSwitch.pressed();
    bool reedReleased = reedSwitch.released();

    // Throttled reed switch diagnostics (every 500ms)
    static unsigned long lastReedLog = 0;
    static RunState lastLoggedState = RunState::Finished; // force first print
    if (now - lastReedLog >= 500 || runState != lastLoggedState) {
        lastReedLog = now;
        lastLoggedState = runState;
        int rawPin = digitalRead(REED_SWITCH_PIN);
        Serial.printf("[%lu ms] State=%-8s | PIN18 raw=%d (LOW=closed) | isPressed=%d\n",
            now,
            runState == RunState::Idle     ? "Idle"    :
            runState == RunState::Armed    ? "Armed"   :
            runState == RunState::Running  ? "Running" : "Finished",
            rawPin,
            reedSwitch.isPressed()
        );
    }

    // Log edge events immediately
    if (reedPressed)  Serial.printf("[%lu ms] REED EVENT: pressed (magnet applied)\n", now);
    if (reedReleased) Serial.printf("[%lu ms] REED EVENT: released (magnet removed)\n", now);

    switch (runState) {

        case RunState::Idle:
            if (!wifiConnected) connectToWiFi();
            // Also allow transition if magnet is already present at boot (isPressed covers held state)
            if (reedPressed || reedSwitch.isPressed()) {
                Serial.println(">>> Transitioning Idle -> Armed");
                runState = RunState::Armed;
            }
            break;

        case RunState::Armed:
            WiFi.disconnect();
            wifiConnected = false;
            headingBeep();
            if (reedReleased) {
                Serial.println(">>> Transitioning Armed -> Running");
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