#include "RB3202_lbr.hpp"
#include "RB3202_servo_analog.hpp"
#include <driver/ledc.h>


using namespace rb3202;

static rb3202::stepper_motor m;


extern "C" void app_main()
{
  ServoAnalog servo14(GPIO_NUM_12, LEDC_TIMER_0, LEDC_CHANNEL_0, 90);

  vTaskDelay(2000/portTICK_PERIOD_MS);

  while(true) {
    printf("start while\n");

    vTaskDelay(1000/portTICK_PERIOD_MS);
    for(int i = 0; i <= 180; i+=5) {
      servo14.set(i);
      printf("uhel-%d\n", i);
      vTaskDelay(500/portTICK_PERIOD_MS);
    }

    servo14.set(0);
    vTaskDelay(2000/portTICK_PERIOD_MS);
  }
}
