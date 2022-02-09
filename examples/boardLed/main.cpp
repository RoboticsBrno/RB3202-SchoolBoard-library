#include "RB3202_lbr.hpp"

using namespace rb3202;

extern "C" void app_main()
{ 
    // Configure pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_R_GPIO | 1ULL << LED_G_GPIO | 1ULL << LED_B_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_pad_select_gpio(LED_R_GPIO); // set pin as GPIO 
    gpio_set_direction(LED_R_GPIO, GPIO_MODE_OUTPUT); // set pin as Output 
    gpio_pad_select_gpio(LED_G_GPIO);
    gpio_set_direction(LED_G_GPIO, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(LED_B_GPIO);
    gpio_set_direction(LED_B_GPIO, GPIO_MODE_OUTPUT);

    while(true) {
        gpio_set_level(LED_R_GPIO, 0);
        gpio_set_level(LED_G_GPIO, 0);
        gpio_set_level(LED_B_GPIO, 0);
        vTaskDelay(500 / portTICK_RATE_MS);
        gpio_set_level(LED_R_GPIO, 1);
        gpio_set_level(LED_G_GPIO, 1);
        gpio_set_level(LED_B_GPIO, 1);
        vTaskDelay(500 / portTICK_RATE_MS);

    }
}

