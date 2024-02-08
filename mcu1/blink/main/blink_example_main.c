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
#include "constants.h"

void app_main(void)
{
    // Initialize spi bus as master
    // default transfer size is 64 bytes when DMA disabled
    spi_bus_config_t spi_config = {
        .mosi_io_num = 19, 
        .miso_io_num = 21, 
        .sclk_io_num = 20, 
        .data2_io_num = -1, 
        .data3_io_num = -1, 
        .data4_io_num = -1, 
        .data5_io_num = -1, 
        .data6_io_num = -1, 
        .data7_io_num = -1, 
        .intr_flags = ESP_INTR_FLAG_LOWMED
    }; 
    spi_bus_initialize(SPI1_HOST, &spi_config, SPI_DMA_DISABLED);

    spi_device_interface_config_t dc_mc_config = {
        .command_bits = 8, 
        .address_bits = 8, 
        .mode = 0,
        .clock_speed_hz = 100000, 
        .spics_io_num = 25
    }; 

    spi_device_handle_t dc_mc_spi;

    spi_bus_add_device(SPI1_HOST, &dc_mc_config, &dc_mc_spi);

     spi_device_interface_config_t stp_mc_config = {
        .command_bits = 8, 
        .address_bits = 8, 
        .mode = 0,
        .clock_speed_hz = 100000, 
        .spics_io_num = 30
    }; 

    spi_device_handle_t stp_mc_spi;

    spi_bus_add_device(SPI1_HOST, &stp_mc_config, &stp_mc_spi);
    
    // Initialize i2c bus as slave to listen to jetson nano
    i2c_config_t i2c_jetson_config = {
        .mode = I2C_MODE_SLAVE, 
        .sda_io_num = 17, 
        .scl_io_num = 18, 
        .sda_pullup_en = true, 
        .scl_pullup_en = true, 
        .slave = {
            .addr_10bit_en = 0, 
            .slave_addr = 0x1000, 
            .maximum_speed = 100000
        }
    };
    i2c_param_config(I2C_NUM_0, &i2c_jetson_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_SLAVE,  32, 32, ESP_INTR_FLAG_LOWMED);

    // Initialize i2c bus as master to buck converter and imu
    i2c_config_t i2c_periph_config = {
        .mode = I2C_MODE_MASTER, 
        .sda_io_num = 28, 
        .scl_io_num = 29, 
        .sda_pullup_en = true, 
        .scl_pullup_en = true, 
        .master = {
            .clk_speed = 100000
        }
    };
    i2c_param_config(I2C_NUM_1, &i2c_periph_config);
    i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 32, 32, ESP_INTR_FLAG_LOWMED );

    // some sort of indication mcu1 is set up 
    int DATA_LENGTH = 4;
    uint8_t *data = (uint8_t *)malloc(DATA_LENGTH);
    uint8_t *rx_buf = (uint8_t *)malloc(DATA_LENGTH);
    esp_err_t err;
    while(true){
        bool stat_ok = true;
        // wait until jetson nano reads from mcu1 over i2c 
        // based on jetson nano command, do different tasks
        int readBytes = i2c_slave_read_buffer(I2C_NUM_0, data, DATA_LENGTH, portMAX_DELAY);

        // check if command is complete
        if (readBytes >= DATA_LENGTH ){
            uint8_t tx = data[2];
            uint16_t device = data[0] << 4 | data[1];
            switch (device) {
                case DRIVE_MC: 
                    // TODO: update command and address once electronics are completed
                    spi_transaction_t spi_dc_tx = {
                        .cmd = 1, 
                        .addr = 1, 
                        .length = 3, 
                        .tx_buffer = &tx
                    };
                    err = spi_device_transmit(dc_mc_spi, &spi_dc_tx);

                    if (stat_ok && err != ESP_OK){
                        stat_ok = false;
                        uint32_t error_msg[2] = {SPI_TX_ERR, err};
                        i2c_slave_write_buffer(I2C_NUM_0, *error_msg, sizeof(uint32_t)*2, portMAX_DELAY);
                    }

                    // TODO: update command and addresses to read from the status register
                    spi_transaction_t spi_dc_rx = {
                        .cmd = 1, 
                        .addr = 1, 
                        .length = 3,
                        .rxlength = DATA_LENGTH,
                        .rx_buffer = rx_buf
                    };
                    err = spi_device_transmit(dc_mc_spi, &spi_dc_rx);

                    if (stat_ok && err != ESP_OK){
                        stat_ok = false;
                        uint8_t error_msg[2] = {SPI_RX_ERR, err};
                        i2c_slave_write_buffer(I2C_NUM_0, *error_msg, sizeof(uint8_t)*2, portMAX_DELAY);
                    }

                    if (stat_ok && rx_buf[0]!=MC_STAT_OK) {
                        stat_ok = false;
                        uint8_t error_msg[2] = {DRIVE_MC_ERR, rx_buf};
                        i2c_slave_write_buffer(I2C_NUM_0, *error_msg, sizeof(uint8_t)*2, portMAX_DELAY);
                    }
                break;
                case SERVO_MC:
                    // TODO: update command and address once electronics are completed
                    spi_transaction_t spi_stp_tx = {
                        .cmd = 1, 
                        .addr = 1, 
                        .length = 3, 
                        .tx_buffer = &tx
                    };
                    err = spi_device_transmit(stp_mc_spi, &spi_stp_tx);

                    if (stat_ok && err != ESP_OK){
                        stat_ok = false;
                        uint32_t error_msg[2] = {SPI_TX_ERR, err};
                        i2c_slave_write_buffer(I2C_NUM_0, *error_msg, sizeof(uint32_t)*2, portMAX_DELAY);
                    }

                    // TODO: update command and addresses to read from the status register
                    spi_transaction_t spi_servo_rx = {
                        .cmd = 1, 
                        .addr = 1, 
                        .length = 3,
                        .rxlength = DATA_LENGTH,
                        .rx_buffer = rx_buf
                    };

                    err = spi_device_transmit(stp_mc_spi, &spi_servo_rx);

                    if (stat_ok && err != ESP_OK){
                        stat_ok = false;
                        uint8_t error_msg[2] = {SPI_RX_ERR, err};
                        i2c_slave_write_buffer(I2C_NUM_0, &error_msg, sizeof(uint8_t)*2, portMAX_DELAY);
                    }

                    if (stat_ok && rx_buf[0]!=MC_STAT_OK) {
                        stat_ok = false;
                        uint8_t error_msg[2] = {SERVO_MC_ERR, rx_buf};
                        i2c_slave_write_buffer(I2C_NUM_0, &error_msg, sizeof(uint8_t)*2, portMAX_DELAY);
                    }
                    
                break;
            }
        }

        if (stat_ok) {
            i2c_slave_write_buffer(I2C_NUM_0, STAT_OK, DATA_LENGTH, portMAX_DELAY);
        }
    }
}
