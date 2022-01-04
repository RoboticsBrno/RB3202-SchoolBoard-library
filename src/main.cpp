#include "RB3202_lbr.hpp"
#include "RB3202_servo_analog.hpp"


static rb3202::stepper_motor m;

static void pouzijServo(rb3202::ServoAnalog& s) {
  s.set(45);


}

extern "C" void app_main()
{
  rb3202::ServoAnalog servo(GPIO_NUM_10, LEDC_TIMER_0, LEDC_CHANNEL_0);

  pouzijServo(servo);

  m.setAllPins();
  m.doStep(1000,-50);
  vTaskDelay(1000);
}
