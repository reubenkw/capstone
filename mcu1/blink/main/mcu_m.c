#include <unistd.h>
#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>

#include "mcu_m.h"
#include "i2c.h"
#include "mcu_gpio.h"
#include "spi.h"
#include "mc_drive.h"

// NOTE: integration tests should be standalone and perform all required initialization

void test_drive_one() {
    init_boost();
    // need a delay after boost enable for precharge
    sleep(10);
    init_dc_mc();

    drive(200, 200, 200, 200, false, 10);

    sleep(3);

    drive(200, 200, 200, 200, true, 10);
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

void test_jetson_drive_interface() {
    init_boost();
    // need a delay after boost enable for precharge
    sleep(10);
    init_dc_mc();
    init_i2c_jetson_mcu_m();

    // 5 extra data bytes: uint8_t pwm speed + 4 byte float for time (seconds)
    uint8_t rx_data[5] = {0};
    mcu_e_status_t state = S_M_ACTION_COMPLETE;
    i2c_write_jetson(S_M_ACTION_COMPLETE);
    while(true) {
        // read from jetson
        printf("waiting for command. willing to wait 7 days.\n");
        i2c_slave_read_buffer(I2C_HOST, rx_data, 1, portMAX_DELAY);
        printf("got command: %x\n", rx_data[0]);
        usleep(10000);
        switch (rx_data[0]) {
            case CMD_M_WRITE_STATUS:
                i2c_write_jetson(state);
                break;
            case CMD_M_FWD: {
                state = S_M_PROCESSING_CMD;
                i2c_write_jetson(state);
                i2c_slave_read_buffer(I2C_HOST, rx_data, 5, portMAX_DELAY);

                // interpret data recieved
                uint8_t pwm_speed = rx_data[0];
                float drive_time = uint8_array_to_float(&rx_data[1]);
                printf("drive forward (pwm_speed: %d/255, time: %f [s])\n", pwm_speed, drive_time);

                drive(pwm_speed, pwm_speed, pwm_speed, pwm_speed, true, drive_time);

                state = S_M_ACTION_COMPLETE;
                i2c_write_jetson(state);
                break;
            }
            case CMD_M_BKWD: {
                state = S_M_PROCESSING_CMD;
                i2c_write_jetson(state);
                i2c_slave_read_buffer(I2C_HOST, rx_data, 5, portMAX_DELAY);

                // interpret data recieved
                uint8_t pwm_speed = rx_data[0];
                float drive_time = uint8_array_to_float(&rx_data[1]);
                printf("drive backward (pwm_speed: %d/255, time: %f [s])\n", pwm_speed, drive_time);

                drive(pwm_speed, pwm_speed, pwm_speed, pwm_speed, false, drive_time);

                state = S_M_ACTION_COMPLETE;
                i2c_write_jetson(state);
                break;
            }
        }
        usleep(5000);
    }
}

