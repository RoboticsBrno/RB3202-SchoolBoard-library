#include "RB3202_lbr.hpp"
#include <SmartLeds.h>

using namespace rb3202;

static rb3202::stepper_motor m;


static void smartLedsTest(int r, int g) {
  SmartLed ledBus(LED_WS2812B, 8, GPIO_NUM_14, 0, DoubleBuffer);
  ledBus[0] = Rgb(r, g, 0);

  for(auto& px : ledBus) {
    px = Rgb(r, g, 0);
  }

  ledBus.show();
  ledBus.wait();
}

extern "C" void app_main()
{

  while(true) {
    vTaskDelay(1000/portTICK_PERIOD_MS);
    smartLedsTest(255, 0);
    vTaskDelay(1000/portTICK_PERIOD_MS);
    smartLedsTest(255, 255);
    vTaskDelay(1000/portTICK_PERIOD_MS);
    smartLedsTest(0, 255);
  }
}
