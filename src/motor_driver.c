#include "motor_driver.h"

void motor_init() {
    ledc_channel_config_t left_forward = {0}, right_forward = {0}, left_reverse = {0}, right_reverse = {0};
    left_forward.gpio_num = IN2;
    left_forward.speed_mode = LEDC_LOW_SPEED_MODE;
    left_forward.channel = LEFT_FORWARD_PWM_CHANNEL;
    left_forward.intr_type = LEDC_INTR_DISABLE;
    left_forward.timer_sel = LEDC_TIMER_1;
    left_forward.duty = MOTOR_LOGIC_1;

    right_forward.gpio_num = IN4;
    right_forward.speed_mode = LEDC_LOW_SPEED_MODE;
    right_forward.channel = RIGHT_FORWARD_PWM_CHANNEL;
    right_forward.intr_type = LEDC_INTR_DISABLE;
    right_forward.timer_sel = LEDC_TIMER_1;
    right_forward.duty = MOTOR_LOGIC_1;

    left_reverse.gpio_num = IN1;
    left_reverse.speed_mode = LEDC_LOW_SPEED_MODE;
    left_reverse.channel = LEFT_REVERSE_PWM_CHANNEL;
    left_reverse.intr_type = LEDC_INTR_DISABLE;
    left_reverse.timer_sel = LEDC_TIMER_1;
    left_reverse.duty = MOTOR_LOGIC_1;

    right_reverse.gpio_num = IN3;
    right_reverse.speed_mode = LEDC_LOW_SPEED_MODE;
    right_reverse.channel = RIGHT_REVERSE_PWM_CHANNEL;
    right_reverse.intr_type = LEDC_INTR_DISABLE;
    right_reverse.timer_sel = LEDC_TIMER_1;
    right_reverse.duty = MOTOR_LOGIC_1;

    ledc_timer_config_t timer = {0};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = 8;
    timer.timer_num = LEDC_TIMER_1;
    timer.freq_hz = 60000;

    ESP_ERROR_CHECK(ledc_channel_config(&left_forward));
    ESP_ERROR_CHECK(ledc_channel_config(&right_forward));
    ESP_ERROR_CHECK(ledc_channel_config(&left_reverse));
    ESP_ERROR_CHECK(ledc_channel_config(&right_reverse));
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    gpio_config_t EN1_pin = {0}, EN2_pin = {0}, EN3_pin = {0}, EN4_pin = {0}, SLEEP_pin = {0};
    EN1_pin.pin_bit_mask = 1 << EN1;
    EN1_pin.mode = GPIO_MODE_OUTPUT;
    EN1_pin.pull_up_en = GPIO_PULLUP_ENABLE;
    EN1_pin.pull_down_en = GPIO_PULLDOWN_DISABLE;
    EN1_pin.intr_type = GPIO_INTR_DISABLE;

    EN2_pin.pin_bit_mask = 1 << EN2;
    EN2_pin.mode = GPIO_MODE_OUTPUT;
    EN2_pin.pull_up_en = GPIO_PULLUP_ENABLE;
    EN2_pin.pull_down_en = GPIO_PULLDOWN_DISABLE;
    EN2_pin.intr_type = GPIO_INTR_DISABLE;

    EN3_pin.pin_bit_mask = 1 << EN3;
    EN3_pin.mode = GPIO_MODE_OUTPUT;
    EN3_pin.pull_up_en = GPIO_PULLUP_ENABLE;
    EN3_pin.pull_down_en = GPIO_PULLDOWN_DISABLE;
    EN3_pin.intr_type = GPIO_INTR_DISABLE;

    EN4_pin.pin_bit_mask = 1 << EN4;
    EN4_pin.mode = GPIO_MODE_OUTPUT;
    EN4_pin.pull_up_en = GPIO_PULLUP_ENABLE;
    EN4_pin.pull_down_en = GPIO_PULLDOWN_DISABLE;
    EN4_pin.intr_type = GPIO_INTR_DISABLE;

    SLEEP_pin.pin_bit_mask = 1 << SLEEP;
    SLEEP_pin.mode = GPIO_MODE_OUTPUT;
    SLEEP_pin.pull_up_en = GPIO_PULLUP_ENABLE;
    SLEEP_pin.pull_down_en = GPIO_PULLDOWN_DISABLE;
    SLEEP_pin.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&EN1_pin));
    ESP_ERROR_CHECK(gpio_config(&EN2_pin));
    ESP_ERROR_CHECK(gpio_config(&EN3_pin));
    ESP_ERROR_CHECK(gpio_config(&EN4_pin));
    ESP_ERROR_CHECK(gpio_config(&SLEEP_pin));

    gpio_set_level(EN1, 1);
    gpio_set_level(EN2, 1);
    gpio_set_level(EN3, 1);
    gpio_set_level(EN4, 1);
    gpio_set_level(SLEEP, 1);
}

static void set_pwm_value(uint8_t pin, uint32_t value) {
    ledc_channel_t channel;
    switch(pin) {
        case IN1:
            channel = LEFT_REVERSE_PWM_CHANNEL;
            break;
        case IN2:
            channel = LEFT_FORWARD_PWM_CHANNEL;
            break;
        case IN3:
            channel = RIGHT_REVERSE_PWM_CHANNEL;
            break;
        case IN4:
            channel = RIGHT_FORWARD_PWM_CHANNEL;
            break;
        default:
            return;
    }
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, value));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

void motor_brake() {
    gpio_set_level(EN1, 1);
    gpio_set_level(EN2, 1);
    gpio_set_level(EN3, 1);
    gpio_set_level(EN4, 1);
    set_pwm_value(IN1, MOTOR_LOGIC_1);
    set_pwm_value(IN2, MOTOR_LOGIC_1);
    set_pwm_value(IN3, MOTOR_LOGIC_1);
    set_pwm_value(IN4, MOTOR_LOGIC_1);
}

void motor_coast() {
    gpio_set_level(EN1, 0);
    gpio_set_level(EN2, 0);
    gpio_set_level(EN3, 0);
    gpio_set_level(EN4, 0);
}

void motor_forward(uint8_t speed) {
    gpio_set_level(EN1, 1);
    gpio_set_level(EN2, 1);
    gpio_set_level(EN3, 1);
    gpio_set_level(EN4, 1);
    set_pwm_value(IN1, MOTOR_LOGIC_1);
    set_pwm_value(IN2, MAX_MOTOR_SPEED - speed);
    set_pwm_value(IN3, MOTOR_LOGIC_1);
    set_pwm_value(IN4, MAX_MOTOR_SPEED - speed);
}

void motor_reverse(uint8_t speed) {
    gpio_set_level(EN1, 1);
    gpio_set_level(EN2, 1);
    gpio_set_level(EN3, 1);
    gpio_set_level(EN4, 1);
    set_pwm_value(IN1, MAX_MOTOR_SPEED - speed);
    set_pwm_value(IN2, MOTOR_LOGIC_1);
    set_pwm_value(IN3, MAX_MOTOR_SPEED - speed);
    set_pwm_value(IN4, MOTOR_LOGIC_1);
}

void motor_right(uint8_t speed) {
    gpio_set_level(EN1, 1);
    gpio_set_level(EN2, 1);
    gpio_set_level(EN3, 1);
    gpio_set_level(EN4, 1);
    set_pwm_value(IN1, MOTOR_LOGIC_1);
    set_pwm_value(IN2, MAX_MOTOR_SPEED - speed);
    set_pwm_value(IN3, MAX_MOTOR_SPEED - speed);
    set_pwm_value(IN4, MOTOR_LOGIC_1);
}

void motor_left(uint8_t speed) {
    gpio_set_level(EN1, 1);
    gpio_set_level(EN2, 1);
    gpio_set_level(EN3, 1);
    gpio_set_level(EN4, 1);
    set_pwm_value(IN1, MAX_MOTOR_SPEED - speed);
    set_pwm_value(IN2, MOTOR_LOGIC_1);
    set_pwm_value(IN3, MOTOR_LOGIC_1);
    set_pwm_value(IN4, MAX_MOTOR_SPEED - speed);
}

void motor_sleep(bool sleep) {
    if (sleep) {
        gpio_set_level(SLEEP, 0);
    } else {
        gpio_set_level(SLEEP, 1);
    }
}