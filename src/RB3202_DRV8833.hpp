#pragma once

#include "RB3202_pinout.hpp"

#define FREGUENCY               3500
#define PWM_RESOLUTION          LEDC_TIMER_8_BIT
#define MAX_PWM                 256

struct driver_pins_t
{
    gpio_num_t pwm_pin[4]
    #if BOARD == RB3202_BOARD
    = {
        rb3202::MOTOR_PWM0_GPIO,
        rb3202::MOTOR_PWM1_GPIO,
        rb3202::MOTOR_PWM2_GPIO,
        rb3202::MOTOR_PWM3_GPIO,
    }
    #endif
    ;

    gpio_num_t sleep_pin = rb3202::MOTOR_SLEEP_GPIO;
};

struct driver_state_t
{
    int pwm_num[4] = {0,0,0,0};
    bool speel_mode = 0;
};

namespace rb3202
{
class DRV8833
{
private:
int setPwmPercent(float percent, int channel);


protected:
void setMotorPwmPins(gpio_num_t pin, uint8_t channel);
void setMotorSleepPin(gpio_num_t pin);
void setPins(driver_pins_t pins);


public:
void setDriver();
void internalSetChannelPower(bool channel, int power);
void setChannelPower(bool channel, float power);

};
}