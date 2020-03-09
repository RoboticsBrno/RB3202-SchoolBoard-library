#include "RB3202_lbr.hpp"

void setup()
{
  ui.sed_periphery(); // nazacatku musime inicializovat uzivatelske rozhrani (tlacitka a ledky)
  ui.led_state(LED_R, HIGH);  // rozsvitime cervenou jako vychozi stav semaforu
}

void loop()
{
  if(ui.read_button(SW0))   // pokud stiskneme tlacitko SW0 zacneme prepinat semafor
  {
    ui.led_state(LED_R,  LOW);  // zhasneme cervenou
    ui.led_state(LED_B, HIGH);  // rozsvitime oranzovou (ja rozsvesim modrou protoze mem starou desku kde oranzova neni)
    delay(1000);                // pockame 1 sekundu (1000 milisekund) nez budeme pokracovat
    ui.led_state(LED_B,  LOW);  // zhasneme oranzovou
    ui.led_state(LED_G, HIGH);  // rozsvitime zelenou
    Serial.println("SW0");
  }
  else if(ui.read_button(SW1))  // pokud stiskneme tlacitko SW1 zacneme prepinat semafor
  {
    ui.led_state(LED_B, HIGH);  // rozsvitime oranzovou (ja rozsvesim modrou protoze mem starou desku kde oranzova neni)
    delay(1000);                // pockame 1 sekundu (1000 milisekund) nez budeme pokracovat
    ui.led_state(LED_G,  LOW);  // zhasneme zelenou
    ui.led_state(LED_B,  LOW);  // zhasneme oranzovou
    ui.led_state(LED_R, HIGH);  // rozsvitime cervenou
    Serial.println("SW1");
  }
  delay(100);   // protoze nemusime cist tlacitka a nastoavoval ledky az tak casto chvili pockame
}