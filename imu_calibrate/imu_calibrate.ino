#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <SPI.h>

// --- SPI Pin Definitions ---
#define BNO08X_SCK   7
#define BNO08X_MISO  15
#define BNO08X_MOSI  5
#define BNO08X_CS    4
#define BNO08X_INT   16
#define BNO08X_RST   6

Adafruit_BNO08x bno08x(BNO08X_RST);
sh2_SensorValue_t sensorValue;

// Calibration status tracking
uint8_t calAccel = 0, calGyro = 0, calMag = 0;
bool calibrationComplete = false;
unsigned long lastPrintTime = 0;
unsigned long lastSaveTime = 0;

#define DCD_SAVE_INTERVAL 5000
#define PRINT_INTERVAL 500

void setReports() {
  // Rotation vector fuses all 3 sensors — good overall check
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 50000)) {
    Serial.println("Could not enable rotation vector report");
  }
  // Calibrated mag report — needed so mag calibration runs
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 50000)) {
    Serial.println("Could not enable magnetometer report");
  }
  // Game rotation vector — accel + gyro only
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 50000)) {
    Serial.println("Could not enable game rotation vector");
  }
}

void enableCalibration() {
  // Tell the BNO085 to actively calibrate all 3 sensors
  // SH2_CAL_ACCEL=1, SH2_CAL_GYRO=2, SH2_CAL_MAG=4
  int status = sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);
  if (status == SH2_OK) {
    Serial.println("Dynamic calibration ENABLED for accel, gyro, mag.");
  } else {
    Serial.printf("WARNING: sh2_setCalConfig failed (status %d)\n", status);
  }

  // Disable auto-save so we control when DCD is written
  sh2_setDcdAutoSave(false);
}

void saveDCD() {
  int status = sh2_saveDcdNow();
  if (status == SH2_OK) {
    Serial.println(">> DCD saved to BNO085 internal flash!");
  } else {
    Serial.printf(">> ERROR: DCD save failed (status %d)\n", status);
  }
}

void printCalibrationStatus() {
  Serial.printf("  Accel: %d/3  |  Gyro: %d/3  |  Mag: %d/3", calAccel, calGyro, calMag);
  if (calAccel >= 3 && calGyro >= 3 && calMag >= 3) {
    Serial.print("  <<<  ALL CALIBRATED!");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("\n========================================");
  Serial.println("  BNO085 Calibration & Save Utility");
  Serial.println("========================================\n");

  SPI.begin(BNO08X_SCK, BNO08X_MISO, BNO08X_MOSI, BNO08X_CS);

  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT, &SPI)) {
    Serial.println("ERROR: Failed to find BNO085 over SPI. Check wiring!");
    while (1) delay(100);
  }

  Serial.println("BNO085 found over SPI!\n");

  enableCalibration();
  setReports();

  Serial.println("\n--- CALIBRATION INSTRUCTIONS ---");
  Serial.println("1. GYROSCOPE:  Leave sensor still on flat surface ~3 sec");
  Serial.println("2. ACCELEROMETER: Slowly place each of 6 faces pointing up");
  Serial.println("3. MAGNETOMETER: Slow figure-8 motions in the air");
  Serial.println("");
  Serial.println("Accuracy: 0=unreliable, 3=fully calibrated");
  Serial.println("Type 's' to force-save, 'r' to reset calibration.\n");
}

void loop() {
  // Serial commands
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's' || c == 'S') {
      Serial.println("\n>> Manual save requested...");
      saveDCD();
    } else if (c == 'r' || c == 'R') {
      Serial.println("\n>> Resetting calibration...");
      sh2_clearDcdAndReset();
      delay(500);
      if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT, &SPI)) {
        Serial.println("ERROR: Reinit failed after reset!");
        while (1) delay(100);
      }
      enableCalibration();
      setReports();
      calibrationComplete = false;
      calAccel = calGyro = calMag = 0;
      Serial.println(">> Calibration cleared. Start moving the sensor.\n");
    }
  }

  // Read sensor events and extract accuracy from status field
  if (bno08x.getSensorEvent(&sensorValue)) {
    // The status field's lower 2 bits hold accuracy (0-3)
    uint8_t accuracy = sensorValue.status & 0x03;

    switch (sensorValue.sensorId) {
      case SH2_ROTATION_VECTOR:
        // Rotation vector fuses everything — we track it but use
        // the individual reports for per-sensor accuracy
        break;

      case SH2_MAGNETIC_FIELD_CALIBRATED:
        calMag = accuracy;
        break;

      case SH2_GAME_ROTATION_VECTOR:
        // Game RV = accel + gyro fusion
        calAccel = accuracy;
        calGyro  = accuracy;
        break;
    }
  }

  unsigned long now = millis();

  // Print status periodically
  if (now - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = now;
    printCalibrationStatus();
  }

// Auto-save DCD periodically once partially calibrated
  if (now - lastSaveTime >= DCD_SAVE_INTERVAL) {
    lastSaveTime = now;
    if (!calibrationComplete && (calAccel >= 2 || calGyro >= 2 || calMag >= 2)) {
      saveDCD();
    }
  }

  // Final save when fully calibrated
  if (!calibrationComplete && calAccel >= 3 && calGyro >= 3 && calMag >= 3) {
    calibrationComplete = true;
    Serial.println("\n==========================================");
    Serial.println("  ALL SENSORS FULLY CALIBRATED (3/3)!");
    Serial.println("  Saving final calibration to flash...");
    Serial.println("==========================================\n");
    saveDCD();
    Serial.println("Calibration persists across power cycles.");
    Serial.println("You can now use the sensor normally.\n");
  }
}