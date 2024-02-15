/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/portmacro.h"

#define DRIVE_LB_A 22
#define DRIVE_LB_B 23
#define DRIVE_LB_X 24

#define DRIVE_RF_A 25
#define DRIVE_RF_B 28
#define DRIVE_RF_X 29

#define DRIVE_RB_A 30
#define DRIVE_RB_B 31
#define DRIVE_RB_X 36

#define DRIVE_LF_A 37 
#define DRIVE_LF_B 38
#define DRIVE_LF_X 39

#define SERVO_X_A 4
#define SERVO_X_B 5
#define SERVO_X_X 6

#define SERVO_Y_A 8 
#define SERVO_Y_B 9 
#define SERVO_Y_X 10 

#define SERVO_Z_A 19
#define SERVO_Z_B 20 
#define SERVO_Z_X 21

#define GPIO_OUTPUT_PIN_SEL  ( \
    (1ULL<<DRIVE_LB_A) | (1ULL<<DRIVE_LB_B) | (1ULL<<DRIVE_LB_X) | \
    (1ULL<<DRIVE_LF_A) | (1ULL<<DRIVE_LF_B) | (1ULL<<DRIVE_LF_X) | \
    (1ULL<<DRIVE_RB_A) | (1ULL<<DRIVE_RB_B) | (1ULL<<DRIVE_RB_X) | \
    (1ULL<<DRIVE_RF_A) | (1ULL<<DRIVE_RF_B) | (1ULL<<DRIVE_RF_X) | \
    (1ULL<<SERVO_X_A) | (1ULL<<SERVO_X_B) | (1ULL<<SERVO_X_X) | \
    (1ULL<<SERVO_Y_A) | (1ULL<<SERVO_Y_B) | (1ULL<<SERVO_Y_X) | \
    (1ULL<<SERVO_Z_A) | (1ULL<<SERVO_Z_B) | (1ULL<<SERVO_Z_X) \
)

struct encoder_t {
    int lastState;
    unsigned int pin_a;
    unsigned int pin_b;
};

void test_i2c(){
    i2c_config_t i2c_jetson_config = {
        .mode = I2C_MODE_SLAVE, 
        .sda_io_num = 18, 
        .scl_io_num = 17, 
        .sda_pullup_en = true, 
        .scl_pullup_en = true, 
        .slave = {
            .addr_10bit_en = 0, 
            .slave_addr = 0x2000, 
            .maximum_speed = 100000
        }
    };
    i2c_param_config(I2C_NUM_0, &i2c_jetson_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_SLAVE,  32, 32, ESP_INTR_FLAG_LOWMED);

    uint16_t data[2] = {0xCA, 0xFE};

    while(true){
        i2c_slave_write_buffer(I2C_NUM_0, &data, sizeof(data)*8, portMAX_DELAY);
    }
    
}

void app_main(void)
{
    test_i2c();
    // Initialize i2c bus as slave to listen to jetson nano
    i2c_config_t i2c_jetson_config = {
        .mode = I2C_MODE_SLAVE, 
        .sda_io_num = 18, 
        .scl_io_num = 17, 
        .sda_pullup_en = true, 
        .scl_pullup_en = true, 
        .slave = {
            .addr_10bit_en = 0, 
            .slave_addr = 0x2000, 
            .maximum_speed = 100000
        }
    };
    i2c_param_config(I2C_NUM_0, &i2c_jetson_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_SLAVE,  32, 32, ESP_INTR_FLAG_LOWMED);

    uint16_t encoderCounts[7] = {0, 0, 0, 0, 0, 0, 0};
    struct encoder_t encoders[7];
    encoders[0].lastState = 0;
    encoders[0].pin_a = DRIVE_LB_A;
    encoders[0].pin_b = DRIVE_LB_B;

    encoders[1].lastState = 0;
    encoders[1].pin_a = DRIVE_LF_A;
    encoders[1].pin_b = DRIVE_LF_B;

    encoders[2].lastState = 0;
    encoders[2].pin_a = DRIVE_RB_A;
    encoders[2].pin_b = DRIVE_RB_B;

    encoders[3].lastState = 0;
    encoders[3].pin_a = DRIVE_RF_A;
    encoders[3].pin_b = DRIVE_RF_B;

    encoders[4].lastState = 0;
    encoders[4].pin_a = SERVO_X_A;
    encoders[4].pin_b = SERVO_X_B;

    encoders[5].lastState = 0;
    encoders[5].pin_a = SERVO_Y_A;
    encoders[5].pin_b = SERVO_Y_B;

    encoders[6].lastState = 0;
    encoders[6].pin_a = SERVO_Z_A;
    encoders[6].pin_b = SERVO_Z_B;

    i2c_reset_tx_fifo(I2C_NUM_0);

    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 1;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);


    while(true) {
        i2c_slave_write_buffer(I2C_NUM_0, &encoderCounts, sizeof(encoderCounts)*8, portMAX_DELAY);

        for(int i = 0; i < 7; i++){
            int aState = gpio_get_level(encoders[i].pin_a);
            // If the previous and the current state of the outputA are different, that means a Pulse has occured
            if (aState != encoders[i].lastState){     
                // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
                if (gpio_get_level(encoders[i].pin_b) != aState) { 
                    encoderCounts[i]++;
                } else {
                    encoderCounts[i]--;
                }
            }
            encoders[i].lastState = aState;
        }
    }
}
