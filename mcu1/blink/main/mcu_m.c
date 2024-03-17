#include <unistd.h>
#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>

#include "mcu_m.h"
#include "i2c.h"
#include "mcu_gpio.h"
#include "spi.h"
#include "mc_drive.h"

// NOTE: integration tests should be standalone and perform all required initialization
void test_drive_alternating() {
    init_boost();
    init_spi();

    init_dc_mc();

    drive_alternate_direction();
}

void test_drive_full() {
    init_boost();
    // init_spi();

    // init_dc_mc();

    // drive_full_forward();
    while(true) {
        // check_and_clear_fault();
        usleep(10000);
    }
}

void test_i2c_drive_interface() {
    init_i2c_jetson_mcu_m();
    printf("done init\n");

    uint8_t rx_data[I2C_DATA_LENGTH + 1] = {0};
    while(true){
        // wait until jetson nano reads from mcu1 over i2c 
        // based on jetson nano command, do different tasks
        printf("waiting for command. willing to wait 7 days.\n");
        i2c_slave_read_buffer(I2C_HOST, rx_data, I2C_DATA_LENGTH + 1, portMAX_DELAY);
        dev_led_set_color(0, 50, 50);
        printf("read from jetson: %d %d %d %d \n", rx_data[1], rx_data[2], rx_data[3], rx_data[4]); 
    }
}

void test_jetson_ctrl() {
    init_dev_led();
    dev_led_set_color(50, 0, 0);
    init_i2c_jetson_mcu_m();

    uint8_t rx_data[I2C_DATA_LENGTH + 1] = {0};
    while(true){
        // wait until jetson nano reads from mcu1 over i2c 
        // based on jetson nano command, do different tasks
        i2c_slave_read_buffer(I2C_HOST, rx_data, I2C_DATA_LENGTH + 1, portMAX_DELAY);
        printf("read from jetson: %d %d %d %d", rx_data[1], rx_data[2], rx_data[3], rx_data[4]); 
        switch (rx_data[ADDR_INDEX]) {
            case 0x05: // expected data
                dev_led_set_color(0, 50, 0);
            break;
        }
    }
}

void test_hello_world() {
    while(1){
        printf("hello world2\n");
        printf("%x", 6);
    }
}

void test_mc() {
    init_spi();

    uint8_t tx_buf = 1;
    gpio_set_level(LED_2, 1);
    while (1){
        write_spi(spi_mc_dc_handle, 0x07, tx_buf);
        read_spi(spi_mc_dc_handle, 0x0);
    }
}
