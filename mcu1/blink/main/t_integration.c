#include <unistd.h>
#include "driver/gpio.h"

#include "t_integration.h"
#include "i2c.h"
#include "mcu_gpio.h"
#include "spi.h"

// NOTE: integration tests should be standalone and perform all required initialization

void test_i2c_drive_interface() {
    init_i2c_jetson();
    printf("done init\n");

    uint8_t rx_data[I2C_DATA_LENGTH + 1] = {0};
    while(true){
        // wait until jetson nano reads from mcu1 over i2c 
        // based on jetson nano command, do different tasks
        printf("waiting for command. willing to wait 7 days.\n");
        i2c_slave_read_buffer(I2C_HOST, rx_data, I2C_DATA_LENGTH + 1, portMAX_DELAY);
        dev_led_set_color(0, 50, 50);
        printf("read from jetson: %d %d %d %d \n", rx_data[1], rx_data[2], rx_data[3], rx_data[4]); 
        switch (rx_data[ADDR_INDEX]) {
            case DRIVE_MC: 
                printf("incorrect command requesting drive mc!\n");
            break;
            case SERVO_MC:
                printf("servo_mc\n"); 
                uint32_t delay = 10;
                if (rx_data[STP_X] == GO_CMD){
                    printf("GO_CMD recieved.\n"); 
                    while(i2c_slave_read_buffer(I2C_HOST, rx_data, I2C_DATA_LENGTH + 1, delay) == 0){
                        printf("Going forward...\n"); 
                        usleep(100000);
                    }
                } else if (rx_data[STP_X] == BKWD_CMD) {
                    printf("BKWD_CMD recieved.\n");
                    while(i2c_slave_read_buffer(I2C_HOST, rx_data, I2C_DATA_LENGTH + 1, delay) == 0){
                        printf("Going backward...\n"); 
                        usleep(100000);
                    }
                }
                else {
                    printf("incorrect command requesting not x step!\n");
                }
            break;
        }
    }
}

void test_jetson_ctrl() {
    init_dev_led();
    dev_led_set_color(50, 0, 0);
    init_i2c_jetson();

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

#define MSTEP_DIR 17
#define MSTEP_PULSE 18

void test_microstep_drive_i2c() {
    init_dev_led();
    // red means stop, green go, blue back
    dev_led_set_color(0, 0, 0);
    usleep(5000000);
    dev_led_set_color(50, 0, 50);
    init_i2c_jetson();
    printf("done init\n");

    uint8_t rx_data[I2C_DATA_LENGTH + 1] = {0};
    while(true){
        // wait until jetson nano reads from mcu1 over i2c 
        // based on jetson nano command, do different tasks
        i2c_slave_read_buffer(I2C_HOST, rx_data, I2C_DATA_LENGTH + 1, portMAX_DELAY);
        dev_led_set_color(0, 50, 50);
        printf("read from jetson: %d %d %d %d", rx_data[1], rx_data[2], rx_data[3], rx_data[4]); 
        switch (rx_data[ADDR_INDEX]) {
            case DRIVE_MC: 
                printf("incorrect command requesting drive mc!\n");
            break;
            case SERVO_MC:
                printf("servo_mc"); 
                uint32_t delay = 10;
                if (rx_data[STP_X] == GO_CMD){
                    dev_led_set_color(0, 50, 0);
                    gpio_set_level(MSTEP_DIR, 1);
                    while(i2c_slave_read_buffer(I2C_HOST, rx_data, I2C_DATA_LENGTH + 1, delay) == 0){
                        gpio_set_level(MSTEP_PULSE, 1);
                        usleep(1000);
                        gpio_set_level(MSTEP_PULSE, 0);
                        usleep(1000);
                    }
                } else if (rx_data[STP_X] == BKWD_CMD) {
                    dev_led_set_color(0, 0, 50);
                    gpio_set_level(MSTEP_DIR, 0);
                    while(i2c_slave_read_buffer(I2C_HOST, rx_data, I2C_DATA_LENGTH + 1, delay) == 0){
                        gpio_set_level(MSTEP_PULSE, 1);
                        usleep(1000);
                        gpio_set_level(MSTEP_PULSE, 0);
                        usleep(1000);
                    }
                }
                else {
                    printf("incorrect command requesting not x step!\n");
                }
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
