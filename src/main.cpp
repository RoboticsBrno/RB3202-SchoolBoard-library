#include "RB3202_DRV8833.hpp"
#include "RB3202_dc_motor.hpp"
#include "RB3202_pinout.hpp"

rb3202::DC_motor pid;

extern "C" void app_main()
{
  pid.sedPID(0);
  pid.sedRotate(2);
}