#include "driver/gpio.h"

#include "spi.h"
#include "mcu_gpio.h"

spi_device_handle_t spi_mc_dc_handle;

void init_spi() {
    spi_bus_config_t spi_config = {
        .mosi_io_num = 11, 
        .miso_io_num = 13, 
        .sclk_io_num = 12, 
        .data2_io_num = -1, 
        .data3_io_num = -1, 
        .data4_io_num = -1, 
        .data5_io_num = -1, 
        .data6_io_num = -1, 
        .data7_io_num = -1, 
        .intr_flags = ESP_INTR_FLAG_LOWMED
    }; 
    esp_err_t err = spi_bus_initialize(SPI_HOST, &spi_config, SPI_DMA_DISABLED);
    if (err != ESP_OK){
        printf("err spi init: %d\n", err);
        gpio_set_level(LED_3, 1);
    }
    printf("spi initialized. adding devices.\n");
    // spi_device_interface_config_t spi_mc_stepper_config = {
    //     .command_bits = 2, 
    //     .address_bits = 6, 
    //     .mode = 1,
    //     .clock_speed_hz = (5 * 1000 * 1000), 
    //     .queue_size = 1,
    //     .spics_io_num = 37
    // }; 

    // // Add stepper motor controller as spi device
    // spi_bus_add_device(SPI_HOST, &spi_mc_stepper_config, &spi_mc_stepper_handle);

    spi_device_interface_config_t spi_mc_dc_config = {
        .command_bits = 2, 
        .address_bits = 6, 
        .mode = 1,
        .clock_speed_hz = (5 * 1000 * 1000), 
        .queue_size = 1,
        .spics_io_num = 48
    };

    // Add DC motor controller as spi device
    spi_bus_add_device(SPI_HOST, &spi_mc_dc_config, &spi_mc_dc_handle);
    printf("devices added.\n");
}

uint8_t read_spi(spi_device_handle_t device, uint addr) {
    uint8_t rx_buf;
    spi_transaction_t spi_rx = {
        .cmd = READ, 
        .addr = addr, 
        .length = 8,
        .rxlength = 8,
        .rx_buffer = &rx_buf
    };
    esp_err_t err = spi_device_transmit(device, &spi_rx);
    if (err != ESP_OK){
        printf("err spi read: %d\n", err);
        gpio_set_level(LED_3, 1);
    } else {
        printf("spi read: %d\n", rx_buf & 0xFF );
    }
    return rx_buf;
}

void write_spi(spi_device_handle_t device, uint addr, uint8_t tx_data) {
    spi_transaction_t spi_tx = {
            .cmd = WRITE, 
            .addr = addr, 
            .length = 8, 
            .tx_buffer = &tx_data
        };
    esp_err_t err = spi_device_transmit(device, &spi_tx);
    if (err != ESP_OK){
        printf("err spi write: %d\n", err);
        gpio_set_level(LED_3, 1);
    } else {
        printf("spi write: %d\n", tx_data);
    }
}
