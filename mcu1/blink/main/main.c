/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <unistd.h>

#include "freertos/portmacro.h"
// #include "esp_log.h"

#include "i2c.h"
#include "spi.h"
#include "mc_drive.h"
#include "gpio.h"
#include "t_integration.h"

void final_main() {

    initialize_led();
    gpio_set_level(LED_1, 1);

    // Initialize spi bus as master
    init_boost();
    init_spi();
    
    // Initialize i2c bus as slave to listen to jetson nano
    init_limit_gpio();
    init_i2c_jetson();

    printf("mcu initialized"); 
    gpio_set_level(LED_2, 1);

    // ignore initial byte
    uint8_t rx_data[I2C_DATA_LENGTH + 1] = {0};
    while(true) {
        // wait until jetson nano reads from mcu1 over i2c 
        // based on jetson nano command, do different tasks
        i2c_slave_read_buffer(I2C_HOST, rx_data, I2C_DATA_LENGTH + 1, portMAX_DELAY);
        printf("read from jetson: %d %d %d %d", rx_data[1], rx_data[2], rx_data[3], rx_data[4]);
        
        uint8_t limitVal = 0;
        for(uint8_t i = 0; i < sizeof(limit_pins)/sizeof(limit_pins[0]); i++){
            uint8_t level = gpio_get_level(limit_pins[i]);
            limitVal = limitVal | (level << i);
        }
        // need to pad an extra zero
        uint8_t tx_data[2] = {limitVal, 0x0};
        printf("jetson write: %d", limitVal); 
        i2c_slave_write_buffer(I2C_HOST, tx_data, 2, portMAX_DELAY);
    }
}

void app_main(void)
{
    test_i2c_drive_interface();
    init_spi();
    init_i2c_jetson();
    init_dev_led();
}
