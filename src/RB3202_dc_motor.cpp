#include "RB3202_dc_motor.hpp"
#include "RB3202_pinout.hpp"
#include "RB3202_DRV8833.hpp"
#include "RB3202_encoder.hpp"

namespace rb3202
{
//private:
void DC_motor::PIDProces(void *)
{
    const TickType_t x_delay = COUNT_PID_PERIOD / portTICK_PERIOD_MS;
    RB3202_encoder encoder (pid_data.enc_pin[0], pid_data.enc_pin[1]);
    encoder.init();
    while (true)
    {
        pid_data.enc_position = encoder.getCount();
        setWheelPower();
        vTaskDelay(x_delay);
    }
}

void DC_motor::choicePins()
{
    if(pid_data.motor == 0)
    {
        pid_data.enc_pin[0] = rb3202::ENC_A0_GPIO;
        pid_data.enc_pin[1] = rb3202::ENC_B0_GPIO;
    }
    else if(pid_data.motor == 1)
    {
        pid_data.enc_pin[0] = rb3202::ENC_A1_GPIO;
        pid_data.enc_pin[1] = rb3202::ENC_B1_GPIO;
    }
}

void DC_motor::setWheelPower()
{
    pid_data.en = pid_data.enc_position - pid_data.virtual_wheel;
    pid_data.power += count_P()+count_I()+count_D();
    internalSetChannelPower(pid_data.motor, pid_data.power);
}

float DC_motor::count_P()
{
    return pid_data.en * pid_data.P;
}

float DC_motor::count_I()
{
    float I_membr = 0;
    for(int a = 1000/COUNT_PID_PERIOD; a > 0;a--)
    {
        pid_data.I_memori[a] += pid_data.I_memori[a-1];
    }
    pid_data.I_memori[0] = pid_data.en;
    for(int a = 0; a < 1000/COUNT_PID_PERIOD;a++)
    {
        I_membr += pid_data.I_memori[a];
    }
    return I_membr * pid_data.I;
}

float DC_motor::count_D()
{
    return (pid_data.en - pid_data.I_memori[1])*pid_data.D;
}

void DC_motor::rotateWirtualWheel(void *)
{
    const TickType_t x_delay = COUNT_PID_PERIOD / portTICK_PERIOD_MS;
    while (true)
    {
        pid_data.virtual_wheel += ((pid_data.rotate*COUNT_PID_PERIOD)/1000)*NUMBER_OF_PULS_PER_REVOLUTION;
        vTaskDelay(x_delay);
    }
    
}
//protected:
void DC_motor::adjustConstants(pid_data_t data)
{
    pid_data.P = data.P;
    pid_data.I = data.I;
    pid_data.D = data.D;
}

pid_data_t DC_motor::getPidData()
{
    return pid_data;
}

//public:
void DC_motor::sedRotate(float rotate)
{
    pid_data.rotate = rotate;
}

void DC_motor::sedPID(int motor)
{
    pid_data.motor = motor;
    choicePins();
    setDriver();
    TaskHandle_t xHandle = NULL;
    static uint8_t taskParam;
    DC_motor start;
    xTaskCreate(start.PIDProces , "PID", 10, &taskParam, 1, &xHandle);
    xTaskCreate(start.rotateWirtualWheel , "rotateWirWheel", 10, &taskParam, 1, &xHandle);
}

} // namespace rb3202