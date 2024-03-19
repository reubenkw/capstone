#include <unistd.h>

#include "i2c.h"

void init_i2c_jetson(uint mcu_address) {
    // both sda scl pins are the same
    i2c_config_t i2c_jetson_config = {
        .mode = I2C_MODE_SLAVE, 
        .sda_io_num = 10, 
        .scl_io_num = 9, 
        .sda_pullup_en = true, 
        .scl_pullup_en = true, 
        .slave = {
            .addr_10bit_en = 0, 
            .slave_addr = mcu_address, 
            .maximum_speed = 400000
        }
    };
    i2c_param_config(I2C_HOST, &i2c_jetson_config);
    
    esp_err_t err = i2c_driver_install(I2C_HOST, I2C_MODE_SLAVE, 128, 128, ESP_INTR_FLAG_LOWMED);

    if (err != ESP_OK){
        printf("err driver install: %d\n", err);  
    }
    printf("jetson i2c init\n"); 
}

void init_i2c_jetson_mcu_m() {
    init_i2c_jetson(MCU_M_ADDRESS);
}

void init_i2c_jetson_mcu_e() {
    init_i2c_jetson(MCU_E_ADDRESS);
}

void i2c_write_jetson(mcu_e_status_t status) {
    uint8_t data[1] = {status};
    i2c_reset_tx_fifo(I2C_HOST);
    i2c_slave_write_buffer(I2C_HOST, data, 1, portMAX_DELAY);
    usleep(10000);
    printf("i2c_write_jetson %x.\n", status);
}

void test_i2c_write(uint mcu_address) {
    // Initialize i2c bus as slave to listen to jetson nano
    init_i2c_jetson(mcu_address);
    uint8_t data[2] = {0x0, 0x0};
    while (1) {
        data[0] += 1;
        int data_trans = i2c_slave_write_buffer(I2C_HOST, data, 2, portMAX_DELAY);
        printf("data trans: %d, data: %x\n", data_trans, data[0]);
        usleep(100000);
    }
}

void test_i2c_read(uint mcu_address) {
    init_i2c_jetson(mcu_address);
    uint8_t data[5] = {0, 0, 0, 0, 0};
    while(1){
        i2c_slave_read_buffer(I2C_HOST, data, 5, portMAX_DELAY);
        for (int i = 1; i < 5; i++){
            printf("%x", data[i]);
        }
        printf("\n");
    }
}
