#include "RB3202_lbr.hpp"

extern "C" void app_main()
{
  motorA.sedPID(1);
  motorA.sedRotate(1.0);

  vTaskDelay(100000);
}