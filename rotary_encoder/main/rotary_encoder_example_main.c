/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "driver/i2c.h"

static const char *TAG = "example";

// i2c interface: recieves 1 byte command (only first bit used). 
// first bit describes if the request is a clear counters (1) or data (0)
// clear: all other bits are ignored and all counters reset
// data: all counters put into send q
#define I2C_HOST I2C_NUM_0
#define I2C_READ_DATA_LENGTH 1
#define I2C_WRITE_DATA_LENGTH 8 // 4 int16
#define I2C_COMMAND_MASK 1

// pcnt counter registers are 16-bit signed integer
// -32,768 to +32,767
#define PCNT_HIGH_LIMIT 32000
#define PCNT_LOW_LIMIT -32000

#define DRIVE_LF_A -1
#define DRIVE_LF_B -1
#define DRIVE_LB_A -1
#define DRIVE_LB_B -1
#define DRIVE_RF_A -1
#define DRIVE_RF_B -1
#define DRIVE_RB_A -1
#define DRIVE_RB_B -1

#define SERVO_X_A -1
#define SERVO_X_B -1
#define SERVO_Y_A -1
#define SERVO_Y_B -1
#define SERVO_Z_A -1
#define SERVO_Z_B -1

#define NUM_MOTORS_DRIVE 4
#define NUM_MOTORS_ARM 3
// Need all 4 for drive motors
#define NUM_PCNT_UNITS NUM_MOTORS_DRIVE

// pcnt configs used for all
pcnt_unit_config_t unit_config = {.high_limit = PCNT_HIGH_LIMIT, .low_limit = PCNT_LOW_LIMIT};
pcnt_glitch_filter_config_t filter_config = {.max_glitch_ns = 1000};

// pcnt configs for drive channels
pcnt_chan_config_t chan_config_drive[NUM_MOTORS_DRIVE] = {
    {.edge_gpio_num = DRIVE_LF_A, .level_gpio_num = DRIVE_LF_B}, 
    {.edge_gpio_num = DRIVE_LB_A, .level_gpio_num = DRIVE_LB_B}, 
    {.edge_gpio_num = DRIVE_RF_A, .level_gpio_num = DRIVE_RF_B}, 
    {.edge_gpio_num = DRIVE_RB_A, .level_gpio_num = DRIVE_RB_B},
};

// pcnt configs for arm channels
pcnt_chan_config_t chan_config_arm[NUM_MOTORS_ARM] = {
    {.edge_gpio_num = SERVO_X_A, .level_gpio_num = SERVO_X_B}, 
    {.edge_gpio_num = SERVO_Y_A, .level_gpio_num = SERVO_Y_B}, 
    {.edge_gpio_num = SERVO_Z_A, .level_gpio_num = SERVO_Z_B},
};

void init_pcnt(pcnt_unit_handle_t pcnt_units[]) {
    // create pcnt units
    ESP_LOGI(TAG, "install pcnt unit");
    for (int i = 0; i < NUM_PCNT_UNITS; i++){
        ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_units[i]));
    }

    // set glitch filters on each pcnt unit
    ESP_LOGI(TAG, "set glitch filter");
    for(int i = 0; i < NUM_PCNT_UNITS; i++){
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_units[i], &filter_config));
    }

    // install pcnt channels for both a (drive) and b (arm)
    ESP_LOGI(TAG, "install pcnt channels (a)");
    pcnt_channel_handle_t pcnt_chan_a[NUM_MOTORS_DRIVE] = { NULL, NULL, NULL, NULL};
    for (int i = 0; i < NUM_MOTORS_DRIVE; i++) {
        ESP_ERROR_CHECK(pcnt_new_channel(pcnt_units[i], &chan_config_drive[i], &pcnt_chan_a[i]));
    }

    ESP_LOGI(TAG, "install pcnt channels (b)");
    pcnt_channel_handle_t pcnt_chan_b[NUM_MOTORS_ARM] = { NULL, NULL, NULL};
    for (int i = 0; i < NUM_MOTORS_ARM; i++) {
        ESP_ERROR_CHECK(pcnt_new_channel(pcnt_units[i], &chan_config_arm[i], &pcnt_chan_b[i]));
    }

    // set edge and level actions for pcnt channels a (drive) and b (arm)
    ESP_LOGI(TAG, "set edge and level actions for pcnt channels (a)");
    for (int i = 0; i < NUM_MOTORS_DRIVE; i++) {
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a[i], PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a[i], PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    }

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels (b)");
    for (int i = 0; i < NUM_MOTORS_ARM; i++) {
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b[i], PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b[i], PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    }

    // start all pcnt units
    for (int i = 0; i < NUM_PCNT_UNITS; i++) {
        ESP_LOGI(TAG, "enable pcnt unit");
        ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_units[i]));
        ESP_LOGI(TAG, "clear pcnt unit");
        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_units[i]));
        ESP_LOGI(TAG, "start pcnt unit");
        ESP_ERROR_CHECK(pcnt_unit_start(pcnt_units[i]));
    }
}

void clear_all_pcnt(pcnt_unit_handle_t pcnt_units[]) {
    for (int i = 0; i < NUM_PCNT_UNITS; i++) {
        pcnt_unit_clear_count(pcnt_units[i]);
    }
}

void init_i2c() {
    i2c_config_t i2c_jetson_config = {
        .mode = I2C_MODE_SLAVE, 
        .sda_io_num = -1, 
        .scl_io_num = -1, 
        .sda_pullup_en = true, 
        .scl_pullup_en = true, 
        .slave = {
            .addr_10bit_en = 0, 
            .slave_addr = 0x16, 
            .maximum_speed = 100000
        }
    };
    i2c_param_config(I2C_HOST, &i2c_jetson_config);

    ESP_ERROR_CHECK(i2c_driver_install(I2C_HOST, I2C_MODE_SLAVE, 128, 128, ESP_INTR_FLAG_LOWMED));
}

// TODO: configure esp logger verbosity after tested
void app_main(void)
{   
    // init i2c
    init_i2c();

    // init pcnt 
    pcnt_unit_handle_t units[NUM_PCNT_UNITS] = {NULL};
    init_pcnt(units);

    // Report counter value
    int pulse_counts[NUM_PCNT_UNITS] = {0};
    while (true) {
        // read i2c. blocks for like 7 days
        uint8_t command[I2C_READ_DATA_LENGTH + 1] = {0};
        // read an extra byte and ignore first (apply mask for first bit)
        if (i2c_slave_read_buffer(I2C_HOST, command, I2C_READ_DATA_LENGTH + 1, portMAX_DELAY) > 0) {
            // data read
            ESP_LOGI(TAG, "Command: %d", command[I2C_READ_DATA_LENGTH] & I2C_COMMAND_MASK);
            // last bit is command
            if (command[I2C_READ_DATA_LENGTH] & I2C_COMMAND_MASK) {
                // reset counters
                for (int i = 0; i < NUM_PCNT_UNITS; i++) {
                    pcnt_unit_clear_count(units[i]);
                }
            } else {
                // send data
                // overflow not considered due to physical limitations of arm and small drive distance
                for (int i = 0; i < NUM_PCNT_UNITS; i++) {
                    ESP_ERROR_CHECK(pcnt_unit_get_count(units[i], &pulse_counts[i]));
                }

                i2c_slave_write_buffer(I2C_HOST,  (uint8_t *)pulse_counts, I2C_WRITE_DATA_LENGTH, portMAX_DELAY);
                // write an extra byte
                i2c_slave_write_buffer(I2C_HOST,  0, I2C_WRITE_DATA_LENGTH, portMAX_DELAY);
                
                ESP_LOGI(TAG, "Pulse counts: (%d, %d, %d, %d)", pulse_counts[0], pulse_counts[1], pulse_counts[2], pulse_counts[3]);
            }
        }
    }
}
