#include "i2c.h"

void init_i2c_jetson() {
    i2c_config_t i2c_jetson_config = {
        .mode = I2C_MODE_SLAVE, 
        .sda_io_num = 10, 
        .scl_io_num = 9, 
        .sda_pullup_en = true, 
        .scl_pullup_en = true, 
        .slave = {
            .addr_10bit_en = 0, 
            .slave_addr = 0x10, 
            .maximum_speed = 400000
        }
    };
    i2c_param_config(I2C_HOST, &i2c_jetson_config);
    
    esp_err_t err = i2c_driver_install(I2C_HOST, I2C_MODE_SLAVE, 128, 128, ESP_INTR_FLAG_LOWMED);

    if (err != ESP_OK){
        printf("err driver install: %d\n", err);  
    }
    printf("jetson i2c init"); 
}

void test_i2c_write() {
    // Initialize i2c bus as slave to listen to jetson nano
    init_i2c_jetson();
    uint8_t data[1] = {0xFF};
    while (1){
        // i2c_reset_tx_fifo(I2C_HOST);
        int data_trans = i2c_slave_write_buffer(I2C_HOST, data, 1, 1000 / portTICK_PERIOD_MS);
        printf("data trans: %d\n", data_trans);
    }
}

void test_i2c_read() {
    init_i2c_jetson();
    uint8_t data[5] = {0, 0, 0, 0, 0};
    while(1){
        i2c_slave_read_buffer(I2C_HOST, data, 5, portMAX_DELAY);
        for (int i = 1; i < 5; i++){
            printf("%x", data[i]);
        }
        printf("\n");
    }
}
