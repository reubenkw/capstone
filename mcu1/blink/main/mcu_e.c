#include <unistd.h>
#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>

#include "i2c.h"
#include "mcu_gpio.h"

// these where for an old test setup
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
