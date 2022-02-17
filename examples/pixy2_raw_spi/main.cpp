#include <freertos/task.h>
#include <vector>
#include <driver/gpio.h>
#include <driver/spi_master.h>

static esp_err_t receiveResponse(spi_device_handle_t spiDev, TickType_t timeout, std::vector<uint8_t>& dest) {

    size_t resp_pos = 0;
    size_t payload_len = 0;
    uint16_t checksum_expected = 0;
    uint16_t checksum_computed = 0;

    const auto timeoutAt = xTaskGetTickCount() + timeout;

    char zeros[8] = { 0 };
    char rxbuff[8] = { 0 };

    while(xTaskGetTickCount() <= timeoutAt) {
        spi_transaction_t trans = {
            .flags = 0,
            .length = sizeof(rxbuff) * 8,
            .rxlength = 0,
            .tx_buffer = zeros,
            .rx_buffer = rxbuff,
        };

        auto err = spi_device_transmit(spiDev, &trans);
        if(err != ESP_OK) {
            return err;
        }

        for(size_t i = 0; i < sizeof(rxbuff); ++i) {
            char in_byte = rxbuff[i];
            printf("%2d: %3d %02x %c p%d\n", i, in_byte, in_byte, in_byte, resp_pos);
            dest.push_back(in_byte);

            switch(resp_pos) {
                case 0: // sync byte
                    if(in_byte != 0xAF) {
                        continue;
                    }
                    break;
                case 1: // sync byte
                    if(in_byte != 0xC1) {
                        --i;
                        resp_pos = 0;
                        dest.clear();
                        continue;
                    }
                    break;
                case 2: // packet type
                    break;
                case 3: // payload len
                    payload_len = in_byte;
                    break;
                case 4: // checksum
                    checksum_computed = 0;
                    checksum_expected = in_byte;
                    break;
                case 5: // checksum
                    checksum_expected |= in_byte << 8;
                    if(payload_len == 0) {
                        return ESP_OK;
                    }
                    break;

                default:
                    checksum_computed += in_byte;
                    --payload_len;
                    if(payload_len == 0) {
                        //printf("%x %x\n", checksum_computed, checksum_expected);
                        return ESP_OK;
                    }
                    break;
            }

            ++resp_pos;
        }
    }

    return ESP_ERR_TIMEOUT;
}

extern "C" void app_main()
{
  const spi_bus_config_t busCfg = {
    .mosi_io_num = GPIO_NUM_13,
    .miso_io_num = GPIO_NUM_12,
    .sclk_io_num = GPIO_NUM_14,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_IOMUX_PINS,
  };

  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &busCfg, 0));

  const spi_device_interface_config_t devCfg = {
    .mode = 3,
    .clock_speed_hz = 2000000,
    .spics_io_num = -1,
    .queue_size = 1,
  };

  spi_device_handle_t spiDev;
  ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devCfg, &spiDev));



  vTaskDelay(pdMS_TO_TICKS(100));

  while(true) {
    char txbuff[] = { 
        0xae,  // first byte of no_checksum_sync (little endian -> least-significant byte first)
        0xc1,  // second byte of no_checksum_sync
        32,
        2,

        1,
        2,
    };

    {
      spi_transaction_t trans = { 
        .length = sizeof(txbuff) * 8,
        .tx_buffer = txbuff,
      };

      ESP_ERROR_CHECK(spi_device_transmit(spiDev, &trans));
    }

    vTaskDelay(pdMS_TO_TICKS(50));


    std::vector<uint8_t> rxbuff;
    if(receiveResponse(spiDev, pdMS_TO_TICKS(100), rxbuff) != ESP_OK) {
        printf("\ntimeout!\n");
        continue;
    }

    if(rxbuff[3] != 14) {
        printf("\rN/A  N/A ");
        continue;
    }

    uint16_t x = rxbuff[8] | rxbuff[9] << 8;
    uint16_t y = rxbuff[10] | rxbuff[11] << 8;
    printf("\r%4d %4d", x, y);

    vTaskDelay(pdMS_TO_TICKS(100));
  }

}

