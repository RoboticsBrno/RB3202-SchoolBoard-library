#include "RB3202_lbr.hpp"
#include "RB3202_servo_analog.hpp"
#include "RB3202_servo_smart.hpp"
#include <driver/ledc.h>


using namespace rb3202;

static rb3202::stepper_motor m;


extern "C" void app_main()
{
  ServoSmartBus bus;
  bus.install(1, UART_NUM_1, GPIO_NUM_35);
  vTaskDelay(2000/portTICK_PERIOD_MS);

  printf("posunuju se\n");
  bus.setAngle(1, Angle::deg(90));


  // auto angle = bus.getAngle(1);
  // printf("Servo angle: %f\n", angle.deg());

  

  // while(true) {
  //   printf("start loop");

  //   vTaskDelay(1000/portTICK_PERIOD_MS);
  //   for(int i = 0; i <= 180; i+=5) {
  //     bus.setAngle(1, Angle::deg(i));
  //     printf("uhel-%d\n", i);
  //     vTaskDelay(500/portTICK_PERIOD_MS);
  //   }

  //   bus.setAngle(1, Angle::deg(0));
  //   vTaskDelay(2000/portTICK_PERIOD_MS);
  // }
}
