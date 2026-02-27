#include "subsystems/Motor.h"

// Standard ESC pulse widths in microseconds
static const int MIN_PULSE_US = 1000;
static const int MAX_PULSE_US = 2000;

Motor::Motor(int motorPin, int ch, int pwmFreq, int pwmResBits)
    : pin(motorPin), channel(ch), frequency(pwmFreq), resolutionBits(pwmResBits), currentSpeed(0.0) {
    maxDuty = (1 << resolutionBits) - 1;
}

void Motor::begin() {
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_conf.timer_num = (ledc_timer_t)channel;
    timer_conf.duty_resolution = (ledc_timer_bit_t)resolutionBits;
    timer_conf.freq_hz = frequency;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ch_conf = {};
    ch_conf.gpio_num = pin;
    ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    ch_conf.channel = (ledc_channel_t)channel;
    ch_conf.timer_sel = (ledc_timer_t)channel;
    ch_conf.duty = 0;
    ch_conf.hpoint = 0;
    ledc_channel_config(&ch_conf);

    stop();
}

void Motor::setSpeed(float speed) {
    if (speed < 0.0) speed = 0.0;
    if (speed > 1.0) speed = 1.0;
    currentSpeed = speed;

    // Convert speed (0.0-1.0) to pulse width (1000us-2000us)
    int pulseUs = MIN_PULSE_US + (int)(speed * (MAX_PULSE_US - MIN_PULSE_US));
    // Convert pulse width to duty cycle value
    int periodUs = 1000000 / frequency;  // 20000us at 50Hz
    uint32_t duty = (uint32_t)((long)pulseUs * maxDuty / periodUs);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel);
}

void Motor::stop() {
    setSpeed(0.0);
}

float Motor::getSpeed() {
    return currentSpeed;
}