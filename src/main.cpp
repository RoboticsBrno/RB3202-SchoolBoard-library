#include "RB3202_lbr.hpp"


using namespace rb3202;

static rb3202::stepper_motor m;


extern "C" void app_main()
{

  ServoSmartBus bus;
  bus.install(1, UART_NUM_1, GPIO_NUM_11);

  auto angle = bus.getAngle(1);
  printf("Servo angle: %f\n", angle.deg());

  //bus.setAngle(1, 120_deg);

  while(true) {
    vTaskDelay(1000);
  }
}