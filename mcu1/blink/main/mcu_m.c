#include <unistd.h>
#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>

#include "mcu_m.h"
#include "i2c.h"
#include "mcu_gpio.h"
#include "spi.h"
#include "mc_drive.h"

// NOTE: integration tests should be standalone and perform all required initialization

void test_drive_full() {
    init_boost();
    init_dc_mc();
    drive_wheels(1.0, 1.0, 1.0, 1.0, 10);
}

void test_drive_one() {
    init_boost();
    init_dc_mc();
    drive_wheels(1.0, 0, 0, 0, 10);
}


void test_i2c_drive_interface() {
    init_i2c_jetson_mcu_m();
    printf("done init\n");

    const uint data_len = 4;
    uint8_t rx_data[data_len];
    while(true){
        // wait until jetson nano reads from mcu1 over i2c 
        // based on jetson nano command, do different tasks
        printf("waiting for command. willing to wait 7 days.\n");
        i2c_slave_read_buffer(I2C_HOST, rx_data, data_len, portMAX_DELAY);
        dev_led_set_color(0, 50, 50);
        printf("read from jetson: %d %d %d %d \n", rx_data[0], rx_data[1], rx_data[2], rx_data[3]); 
    }
}

void test_jetson_ctrl() {
    init_dev_led();
    dev_led_set_color(50, 0, 0);
    init_i2c_jetson_mcu_m();

    const uint data_len = 4;
    uint8_t rx_data[data_len];
    while(true){
        // wait until jetson nano reads from mcu1 over i2c 
        // based on jetson nano command, do different tasks
        i2c_slave_read_buffer(I2C_HOST, rx_data, data_len, portMAX_DELAY);
        printf("read from jetson: %d %d %d %d", rx_data[0], rx_data[1], rx_data[2], rx_data[3]); 
        switch (rx_data[0]) {
            case 0x05: // expected data
                dev_led_set_color(0, 50, 0);
            break;
        }
    }
}

