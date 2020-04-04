#include "RB3202_lbr.hpp"

static rb3202::stepper_motor m;

extern "C" void app_main()
{
  m.setAllPins();
  m.doStep(1000,-50);
  vTaskDelay(1000);
}
