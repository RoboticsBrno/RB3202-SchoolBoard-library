#pragma once

#include "RB3202_pinout.hpp"
#include "RB3202_DRV8833.hpp"
#include "RB3202_encoder.hpp"


#define COUNT_PID_PERIOD                10  // controlled power update period
#define NUMBER_OF_PULS_PER_REVOLUTION   100

namespace rb3202
{

struct pid_data_t
{
    gpio_num_t enc_pin[2];
    int motor = 0;
    float rotate = 0;
    int enc_position = 0;
    int power = 0;
    float virtual_wheel = 0;
    float en = 0;
    float I_memori[1000/COUNT_PID_PERIOD];
    float P = 1;
    float I = 1;
    float D = 1;
};

class DC_motor: DRV8833
{
private:
void PIDProces();
void choicePins();
void setWheelPower();
void rotateWirtualWheel();

float count_P();
float count_I();
float count_D();

protected:
pid_data_t pid_data;
void adjustConstants(pid_data_t data);
pid_data_t getPidData();

public:
void sedPID(int motor);
void sedRotate(float rotate);

};

}