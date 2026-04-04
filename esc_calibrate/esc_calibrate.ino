// ESC Calibration + Motor Test Sketch
// Pins match main project: BL=10, BR=12, TL=9, TR=11
// Steps:
//   1. Flash this sketch with ESCs UNPOWERED
//   2. Open Serial Monitor at 115200
//   3. Follow the prompts

#include "driver/ledc.h"

#define PWM_FREQ     50
#define PWM_RES_BITS 10
#define MAX_DUTY     ((1 << PWM_RES_BITS) - 1)
#define PERIOD_US    (1000000 / PWM_FREQ)  // 20000us

const int MOTOR_PINS[]    = {10, 12, 9, 11};
const char* MOTOR_NAMES[] = {"BL", "BR", "TL", "TR"};
const int NUM_MOTORS = 4;

void setPulse(int channel, int pulseUs) {
    uint32_t duty = (uint32_t)((long)pulseUs * MAX_DUTY / PERIOD_US);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel);
}

void setAllPulse(int pulseUs) {
    for (int i = 0; i < NUM_MOTORS; i++) setPulse(i, pulseUs);
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Init LEDC for all motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        ledc_timer_config_t timer_conf = {};
        timer_conf.speed_mode      = LEDC_LOW_SPEED_MODE;
        timer_conf.timer_num       = (ledc_timer_t)i;
        timer_conf.duty_resolution = (ledc_timer_bit_t)PWM_RES_BITS;
        timer_conf.freq_hz         = PWM_FREQ;
        timer_conf.clk_cfg         = LEDC_AUTO_CLK;
        ledc_timer_config(&timer_conf);

        ledc_channel_config_t ch_conf = {};
        ch_conf.gpio_num   = MOTOR_PINS[i];
        ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
        ch_conf.channel    = (ledc_channel_t)i;
        ch_conf.timer_sel  = (ledc_timer_t)i;
        ch_conf.duty       = 0;
        ch_conf.hpoint     = 0;
        ledc_channel_config(&ch_conf);
    }

    Serial.println("=== ESC CALIBRATION ===");
    Serial.println("STEP 1: Make sure ESCs are UNPOWERED, then press Enter.");
    waitForEnter();

    Serial.println("Sending MAX throttle (2000us)...");
    setAllPulse(2000);

    Serial.println("STEP 2: Power on your ESCs NOW, then wait for the calibration tone, then press Enter.");
    waitForEnter();

    Serial.println("Sending MIN throttle (1050us)...");
    setAllPulse(1050);

    Serial.println("STEP 3: Wait for the confirmation beeps from the ESCs, then press Enter.");
    waitForEnter();

    Serial.println("=== CALIBRATION DONE ===");
    Serial.println();
    Serial.println("=== MOTOR TEST ===");
    Serial.println("Commands:");
    Serial.println("  a <speed>  - all motors (0.0 - 1.0)");
    Serial.println("  0 <speed>  - BL motor");
    Serial.println("  1 <speed>  - BR motor");
    Serial.println("  2 <speed>  - TL motor");
    Serial.println("  3 <speed>  - TR motor");
    Serial.println("  s          - stop all");
    Serial.println();
    Serial.println("WARNING: Remove props before testing!");
}

void loop() {
    if (!Serial.available()) return;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    char key = cmd.charAt(0);

    if (key == 's') {
        setAllPulse(1000);
        Serial.println("All stopped.");
        return;
    }

    if (cmd.length() < 3) {
        Serial.println("Unknown command.");
        return;
    }

    float speed = cmd.substring(2).toFloat();
    speed = constrain(speed, 0.0f, 1.0f);
    int pulseUs = 1000 + (int)(speed * 1000);

    if (key == 'a') {
        setAllPulse(pulseUs);
        Serial.printf("All motors -> %.2f (%d us)\n", speed, pulseUs);
    } else if (key >= '0' && key <= '3') {
        int idx = key - '0';
        setPulse(idx, pulseUs);
        Serial.printf("%s motor -> %.2f (%d us)\n", MOTOR_NAMES[idx], speed, pulseUs);
    } else {
        Serial.println("Unknown command.");
    }
}

void waitForEnter() {
    while (true) {
        if (Serial.available()) {
            Serial.read();
            break;
        }
    }
    delay(100);
    while (Serial.available()) Serial.read(); // flush
}
