#include "subsystems/Motor.h"

namespace {
constexpr int MIN_PULSE_US = 1000;
constexpr int MAX_PULSE_US = 2000;
}

Motor::Motor(uint8_t motorPin, uint8_t ch, uint16_t pwmFreq, uint8_t pwmResBits)
    : pin(motorPin),
      channel(ch),
      frequency(pwmFreq),
      resolutionBits(pwmResBits),
      currentSpeed(0.0f),
      maxDuty((1UL << pwmResBits) - 1UL) {}

void Motor::begin() {
    ledc_timer_config_t timerConf = {};
    timerConf.speed_mode = LEDC_LOW_SPEED_MODE;
    timerConf.timer_num = static_cast<ledc_timer_t>(channel);
    timerConf.duty_resolution = static_cast<ledc_timer_bit_t>(resolutionBits);
    timerConf.freq_hz = frequency;
    timerConf.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timerConf);

    ledc_channel_config_t channelConf = {};
    channelConf.gpio_num = pin;
    channelConf.speed_mode = LEDC_LOW_SPEED_MODE;
    channelConf.channel = static_cast<ledc_channel_t>(channel);
    channelConf.timer_sel = static_cast<ledc_timer_t>(channel);
    channelConf.duty = 0;
    channelConf.hpoint = 0;
    ledc_channel_config(&channelConf);

    stop();
}

void Motor::setSpeed(float speed) {
    speed = constrain(speed, 0.0f, 1.0f);
    currentSpeed = speed;

    const int pulseUs = MIN_PULSE_US + static_cast<int>(speed * static_cast<float>(MAX_PULSE_US - MIN_PULSE_US));
    const int periodUs = 1000000 / frequency;
    const uint32_t duty = static_cast<uint32_t>((static_cast<uint64_t>(pulseUs) * maxDuty) / periodUs);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channel), duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channel));
}

void Motor::stop() {
    setSpeed(0.0f);
}

float Motor::getSpeed() const {
    return currentSpeed;
}
