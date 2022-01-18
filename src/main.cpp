#include "RB3202_lbr.hpp"
#include <SmartLeds.h>


using namespace rb3202;

static rb3202::stepper_motor m;

static void servoTest() {
  ServoSmartBus bus;
  bus.install(1, UART_NUM_1, GPIO_NUM_11);

  auto angle = bus.getAngle(1);
  printf("Servo angle: %f\n", angle.deg());

  //bus.setAngle(1, 120_deg);
}

static void smartLedsTest() {
  SmartLed ledBus(LED_WS2812, 10, GPIO_NUM_12, 0, DoubleBuffer);
  ledBus[0] = Rgb(255, 0, 0);

  for(auto& px : ledBus) {
    px = Rgb(255, 0, 0);
  }

  ledBus.show();
  ledBus.wait();
}

extern "C" void app_main()
{
  servoTest();

  smartLedsTest();
  

  while(true) {
    vTaskDelay(1000);
  }
}
