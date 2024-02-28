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

#include "constants.h"

#define WRITE 0
#define READ 1

#define LIMIT_X_MIN 1
#define LIMIT_X_MAX 1
#define LIMIT_Y_MIN 1
#define LIMIT_Y_MAX 1
#define LIMIT_Z 1

#define GPIO_OUTPUT_PIN_SEL  ( \
    (1ULL<<LIMIT_X_MIN) | (1ULL<<LIMIT_X_MAX) | \
    (1ULL<<LIMIT_Y_MIN) | (1ULL<<LIMIT_Y_MAX) | \
    (1ULL<<LIMIT_Z) \
)

#define SPI_HOST SPI2_HOST
#define I2C_HOST I2C_NUM_0
#define DATA_LENGTH 4

const uint8_t reg_addr_map[4] = {PWM_1, PWM_2, PWM_3, PWM_4};
const uint8_t limit_pins[5] = {LIMIT_X_MIN, LIMIT_X_MAX, LIMIT_Y_MIN, LIMIT_Y_MAX, LIMIT_Z};
uint8_t error[4] = {0};

#define SPI_INIT_ERROR 0
#define SPI_TX_ERROR 1
#define SPI_RX_ERROR 2 

// LED for debugging
#define BLINK_GPIO CONFIG_BLINK_GPIO
static led_strip_handle_t led_strip;

void led_set_color(int r, int g, int b){
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

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
        led_set_color(255, 255, 0);
    }
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
        led_set_color(255, 0, 255);
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
        led_set_color(0, 255, 255);
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

}

void initialize_limit_gpio() {
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
    led_set_color(0, 255, 0);
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

static void configure_led(void)
{
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

void app_main(void)
{
    // Initialize LED for debugging
    configure_led();
    led_set_color(255, 0, 0);

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

    led_set_color(0, 128, 0);

    // ignore initial byte
    uint8_t rx_data[DATA_LENGTH + 1] = {0};
    while(true){
        // wait until jetson nano reads from mcu1 over i2c 
        // based on jetson nano command, do different tasks
        i2c_slave_read_buffer(I2C_HOST, rx_data, DATA_LENGTH + 1, portMAX_DELAY);

        switch (rx_data[ADDR_INDEX]) {
            case DRIVE_MC: 
                if (rx_data[COMMAND_INDEX] == WRITE_CMD){
                    write_spi(dc_mc_spi, rx_data[REG_INDEX], &rx_data[DATA_INDEX]);
                } else {
                    uint8_t read_data = read_spi(dc_mc_spi, rx_data[REG_INDEX]);
                    // need to pad an extra zero
                    uint8_t tx_data[2] = {read_data, 0x0};
                    i2c_slave_write_buffer(I2C_HOST, tx_data, 2, portMAX_DELAY);
                }
            break;
            case SERVO_MC:
                if (rx_data[COMMAND_INDEX] == WRITE_CMD){
                    write_spi(stp_mc_spi, rx_data[REG_INDEX], &rx_data[DATA_INDEX]);
                } else {
                    uint8_t read_data = read_spi(stp_mc_spi, rx_data[REG_INDEX]);
                    // need to pad an extra zero
                    uint8_t tx_data[2] = {read_data, 0x0};
                    i2c_slave_write_buffer(I2C_HOST, tx_data, 2, portMAX_DELAY);
                }
            break;
            case LIMIT: 
                uint8_t limitVal = 0;
                for(uint8_t i = 0; i < sizeof(limit_pins)/sizeof(limit_pins[0]); i++){
                    uint8_t level = gpio_get_level(limit_pins[i]);
                    limitVal = limitVal | (level << i);
                }
                // need to pad an extra zero
                uint8_t tx_data[2] = {limitVal, 0x0};
                i2c_slave_write_buffer(I2C_HOST, tx_data, 2, portMAX_DELAY);
            break;
            case STATUS:
                i2c_slave_write_buffer(I2C_HOST, error, 4, portMAX_DELAY);
                // clear the errors after transmitting
                clear_errors();
            break;
        }
    }
}
