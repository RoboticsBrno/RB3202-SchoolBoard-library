#pragma once

#include <esp_err.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/queue.h>

#include "lx16a/angle.hpp"

namespace rb3202 {

class ServoSmartBus {
public:
    ServoSmartBus();
    ServoSmartBus(const ServoSmartBus&) = delete;
    ~ServoSmartBus();

    esp_err_t install(uint8_t servo_count, uart_port_t uart, gpio_num_t pin);

    void setAngle(uint8_t id, Angle angle);

    Angle getAngle(uint8_t id);

private:
    struct tx_item {
        uint8_t data[16];
        QueueHandle_t response_queue;
        bool expect_response;
        uint8_t size;
    };

    struct rx_response {
        uint8_t data[16];
        uint8_t size;
    };

    void send(uint8_t *data, size_t size);
    struct rx_response sendWithResponseBlocking(uint8_t *data, size_t size, TickType_t timeout = portMAX_DELAY);

    size_t receive(uint8_t *buffer, size_t buffer_capacity);

    static void uartRoutine(void *selfVoid);

    uart_port_t m_uart;
    gpio_num_t m_uart_pin;
    QueueHandle_t m_tx_queue;
};

};
