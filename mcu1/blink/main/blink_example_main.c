/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>

#include "driver/i2c.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "freertos/portmacro.h"
#include "led_strip.h"
#include "esp_log.h"

#include "constants.h"

#define WRITE 0
#define READ 1

#define LIMIT_X_MIN 1
#define LIMIT_X_MAX 1
#define LIMIT_Y_MIN 1
#define LIMIT_Y_MAX 1
#define LIMIT_Z 1

#define GPIO_LIMIT_PIN_SEL  ( \
    (1ULL<<LIMIT_X_MIN) | (1ULL<<LIMIT_X_MAX) | \
    (1ULL<<LIMIT_Y_MIN) | (1ULL<<LIMIT_Y_MAX) | \
    (1ULL<<LIMIT_Z) \
)

#define LED_1 15
#define LED_2 16
#define LED_3 17

#define GPIO_LED_PIN_SEL ( (1ULL<<LED_1) | (1ULL<<LED_2) |(1ULL<<LED_3) )

#define SPI_HOST SPI2_HOST
#define I2C_HOST I2C_NUM_0
#define DATA_LENGTH 4

const uint8_t reg_addr_map[4] = {PWM_1, PWM_2, PWM_3, PWM_4};
const uint8_t limit_pins[5] = {LIMIT_X_MIN, LIMIT_X_MAX, LIMIT_Y_MIN, LIMIT_Y_MAX, LIMIT_Z};
uint8_t error[4] = {0};

#define SPI_INIT_ERROR 0
#define SPI_TX_ERROR 1
#define SPI_RX_ERROR 2 

void initialize_spi() {
    spi_bus_config_t spi_config = {
        .mosi_io_num = 16, 
        .miso_io_num = 17, 
        .sclk_io_num = 8, 
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
        error[SPI_INIT_ERROR] = 1;
        gpio_set_level(LED_3, 1);
    }
    printf("spi initialized"); 
}

void write_spi(spi_device_handle_t device, uint addr, uint8_t * tx_data){
    spi_transaction_t spi_tx = {
            .cmd = WRITE, 
            .addr = addr, 
            .length = 8, 
            .tx_buffer = tx_data
        };
    esp_err_t err = spi_device_transmit(device, &spi_tx);
    if (err != ESP_OK){
        printf("err spi write: %d\n", err);  
        error[SPI_TX_ERROR] = 1;
        gpio_set_level(LED_3, 1);
    } else {
        printf("spi write: %d\n", *tx_data);
    }
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
        error[SPI_RX_ERROR] = 1;
        gpio_set_level(LED_3, 1);
    } else {
        printf("spi read: %d\n", rx_buf );
    }

    printf("data: %d\n", rx_buf);
    return rx_buf;
}

void clear_errors(){
    error[SPI_INIT_ERROR] = 0;
    error[SPI_TX_ERROR] = 0;
    error[SPI_RX_ERROR] = 0;
}

void initialize_i2c_jetson() {
    i2c_config_t i2c_jetson_config = {
        .mode = I2C_MODE_SLAVE, 
        .sda_io_num = 6, 
        .scl_io_num = 5, 
        .sda_pullup_en = true, 
        .scl_pullup_en = true, 
        .slave = {
            .addr_10bit_en = 0, 
            .slave_addr = 0x42, 
            .maximum_speed = 100000
        }
    };
    i2c_param_config(I2C_HOST, &i2c_jetson_config);
    
    esp_err_t err = i2c_driver_install(I2C_HOST, I2C_MODE_SLAVE, 128, 128, ESP_INTR_FLAG_LOWMED);

    if (err != ESP_OK){
        printf("err driver install: %d\n", err);  
    }
    printf("jetson i2c init"); 
}

void initialize_limit_gpio() {
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_LIMIT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 1;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

void test_mc() {
    initialize_spi();

    spi_device_interface_config_t dc_mc_config = {
        .command_bits = 2, 
        .address_bits = 6, 
        .mode = 0,
        .clock_speed_hz = (20 * 1000 * 1000), 
        .queue_size = 1,
        .spics_io_num = 18
    }; 
    spi_device_handle_t dc_mc_spi;
    spi_bus_add_device(SPI_HOST, &dc_mc_config, &dc_mc_spi);

    uint8_t tx_buf = 1;
    gpio_set_level(LED_2, 1);
    while (1){
        write_spi(dc_mc_spi, 0x07, &tx_buf);
        read_spi(dc_mc_spi, 0x0);
    }
}

void test_i2c_write(){
    // Initialize i2c bus as slave to listen to jetson nano
    initialize_i2c_jetson();
    uint8_t data[5] = {0xCA, 0xFE, 0xBA, 0xBE, 0x0};
    while (1){
        int data_trans = i2c_slave_write_buffer(I2C_HOST, data, 5, portMAX_DELAY);
        printf("data trans: %d\n", data_trans);
    }
}

void test_i2c_read(){
    initialize_i2c_jetson();
    uint8_t data[5] = {0, 0, 0, 0, 0};
    while(1){
        i2c_slave_read_buffer(I2C_HOST, data, 5, portMAX_DELAY);
        for (int i = 1; i < 5; i++){
            printf("%x", data[i]);
        }
        printf("\n");
    }
}

void test_hello_world(){
    while(1){
        ESP_LOGI("TEST", "hello world!\n");
        printf("hello world2\n");
        printf("%x", 6);
    }
}

void initialize_led(){
    // LED 
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as input mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_LED_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 1;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

void app_main(void)
{
    initialize_led();
    gpio_set_level(LED_1, 1);

    // Initialize spi bus as master
    initialize_spi();

    // Add DC motor controller as spi device
    spi_device_interface_config_t dc_mc_config = {
        .command_bits = 2, 
        .address_bits = 6, 
        .mode = 0,
        .clock_speed_hz = (20 * 1000 * 1000), 
        .queue_size = 1,
        .spics_io_num = 18
    }; 
    spi_device_handle_t dc_mc_spi;
    spi_bus_add_device(SPI_HOST, &dc_mc_config, &dc_mc_spi);

    // Add stepper motor controller as spi device
    spi_device_interface_config_t stp_mc_config = {
        .command_bits = 2, 
        .address_bits = 6, 
        .mode = 0,
        .clock_speed_hz = (20 * 1000 * 1000), 
        .queue_size = 1,
        .spics_io_num = 35
    }; 
    spi_device_handle_t stp_mc_spi;
    spi_bus_add_device(SPI_HOST, &stp_mc_config, &stp_mc_spi);
    
    // Initialize i2c bus as slave to listen to jetson nano
    initialize_i2c_jetson();
    initialize_limit_gpio();

    printf("mcu initialized"); 
    gpio_set_level(LED_1, 2);

    // ignore initial byte
    uint8_t rx_data[DATA_LENGTH + 1] = {0};
    while(true){
        // wait until jetson nano reads from mcu1 over i2c 
        // based on jetson nano command, do different tasks
        i2c_slave_read_buffer(I2C_HOST, rx_data, DATA_LENGTH + 1, portMAX_DELAY);
        printf("read from jetson: %d %d %d %d", rx_data[1], rx_data[2], rx_data[3], rx_data[4]); 
        switch (rx_data[ADDR_INDEX]) {
            case DRIVE_MC: 
                printf("drive_mc"); 
                if (rx_data[COMMAND_INDEX] == WRITE_CMD){
                    write_spi(dc_mc_spi, rx_data[REG_INDEX], &rx_data[DATA_INDEX]);
                } else {
                    uint8_t read_data = read_spi(dc_mc_spi, rx_data[REG_INDEX]);
                    // need to pad an extra zero
                    uint8_t tx_data[2] = {read_data, 0x0};
                    printf("jetson write: %d", read_data); 
                    i2c_slave_write_buffer(I2C_HOST, tx_data, 2, portMAX_DELAY);
                }
            break;
            case SERVO_MC:
                printf("servo_mc"); 
                if (rx_data[COMMAND_INDEX] == WRITE_CMD){
                    write_spi(stp_mc_spi, rx_data[REG_INDEX], &rx_data[DATA_INDEX]);
                } else {
                    uint8_t read_data = read_spi(stp_mc_spi, rx_data[REG_INDEX]);
                    // need to pad an extra zero
                    uint8_t tx_data[2] = {read_data, 0x0};
                    printf("jetson write: %d", read_data); 
                    i2c_slave_write_buffer(I2C_HOST, tx_data, 2, portMAX_DELAY);
                }
            break;
            case LIMIT: 
                printf("limit"); 
                uint8_t limitVal = 0;
                for(uint8_t i = 0; i < sizeof(limit_pins)/sizeof(limit_pins[0]); i++){
                    uint8_t level = gpio_get_level(limit_pins[i]);
                    limitVal = limitVal | (level << i);
                }
                // need to pad an extra zero
                uint8_t tx_data[2] = {limitVal, 0x0};
                printf("jetson write: %d", limitVal); 
                i2c_slave_write_buffer(I2C_HOST, tx_data, 2, portMAX_DELAY);
            break;
            case STATUS:
                printf("status"); 
                printf("jetson write: %d %d %d %d", error[0], error[1], error[2], error[3]); 
                i2c_slave_write_buffer(I2C_HOST, error, 4, portMAX_DELAY);
                // clear the errors after transmitting
                clear_errors();
            break;
        }
    }
}
