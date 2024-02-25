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

static const char *TAG = "example";

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
pcnt_event_callbacks_t cbs = {.on_reach = example_pcnt_on_reach};

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

static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to queue, from this interrupt callback
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}

void init_pcnt() {
    // create pcnt units
    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_handle_t pcnt_unit[NUM_PCNT_UNITS] = { NULL, NULL, NULL, NULL };
    for (int i = 0; i < NUM_PCNT_UNITS; i++){
        ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit[i]));
    }

    // set glitch filters on each pcnt unit
    ESP_LOGI(TAG, "set glitch filter");
    for(int i = 0; i < NUM_PCNT_UNITS; i++){
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit[i], &filter_config));
    }

    // install pcnt channels for both a (drive) and b (arm)
    ESP_LOGI(TAG, "install pcnt channels (a)");
    pcnt_channel_handle_t pcnt_chan_a[NUM_MOTORS_DRIVE] = { NULL, NULL, NULL, NULL};
    for (int i = 0; i < NUM_MOTORS_DRIVE; i++) {
        ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit[i], &chan_config_drive[i], &pcnt_chan_a[i]));
    }

    ESP_LOGI(TAG, "install pcnt channels (b)");
    pcnt_channel_handle_t pcnt_chan_b[NUM_MOTORS_ARM] = { NULL, NULL, NULL};
    for (int i = 0; i < NUM_MOTORS_ARM; i++) {
        ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit[i], &chan_config_arm[i], &pcnt_chan_b[i]));
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
    QueueHandle_t queue = xQueueCreate(10, sizeof(int));
    for (int i = 0; i < NUM_PCNT_UNITS; i++) {
        ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit[i], &cbs, queue));
        ESP_LOGI(TAG, "enable pcnt unit");
        ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit[i]));
        ESP_LOGI(TAG, "clear pcnt unit");
        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit[i]));
        ESP_LOGI(TAG, "start pcnt unit");
        ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit[i]));
    }
}

// TODO: clear pcnt
void clear_all_pcnt() {
    pcnt_unit_clear_count(pcnt_unit_handle_t unit)
}

void app_main(void)
{
    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config[NUM_MOTORS] = { 
        {.high_limit = PCNT_HIGH_LIMIT, .low_limit = PCNT_LOW_LIMIT}, 
        {.high_limit = PCNT_HIGH_LIMIT, .low_limit = PCNT_LOW_LIMIT}, 
        {.high_limit = PCNT_HIGH_LIMIT, .low_limit = PCNT_LOW_LIMIT}, 
        {.high_limit = PCNT_HIGH_LIMIT, .low_limit = PCNT_LOW_LIMIT}
    };
    pcnt_unit_handle_t pcnt_unit[NUM_MOTORS] = { NULL, NULL, NULL, NULL };

    for (int i = 0; i < NUM_MOTORS; i++){
        ESP_ERROR_CHECK(pcnt_new_unit(&unit_config[i], &pcnt_unit[i]));
    }

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config[NUM_MOTORS] = {
        {.max_glitch_ns = 1000},
        {.max_glitch_ns = 1000},
        {.max_glitch_ns = 1000},
        {.max_glitch_ns = 1000}
    };

    for(int i = 0; i < NUM_MOTORS; i++){
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit[i], &filter_config[i]));
    }

    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config[NUM_MOTORS] = {
        {.edge_gpio_num = DRIVE_LF_A, .level_gpio_num = DRIVE_LF_B}, 
        {.edge_gpio_num = DRIVE_LB_A, .level_gpio_num = DRIVE_LB_B}, 
        {.edge_gpio_num = DRIVE_RF_A, .level_gpio_num = DRIVE_RF_B}, 
        {.edge_gpio_num = DRIVE_RB_A, .level_gpio_num = DRIVE_RB_B} 
    };
    pcnt_channel_handle_t pcnt_chan_a[NUM_MOTORS] = { NULL, NULL, NULL, NULL};

    for (int i = 0; i < NUM_MOTORS; i++){
        ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit[i], &chan_a_config[i], &pcnt_chan_a[i]));
    }

    pcnt_chan_config_t chan_b_config[NUM_MOTORS] = {
        {.edge_gpio_num = DRIVE_LF_B, .level_gpio_num = DRIVE_LF_A}, 
        {.edge_gpio_num = DRIVE_LB_B, .level_gpio_num = DRIVE_LB_A}, 
        {.edge_gpio_num = DRIVE_RF_B, .level_gpio_num = DRIVE_RF_A}, 
        {.edge_gpio_num = DRIVE_RB_B, .level_gpio_num = DRIVE_RB_A}
    };

    pcnt_channel_handle_t pcnt_chan_b[NUM_MOTORS] = { NULL, NULL, NULL, NULL};

    for (int i = 0; i < NUM_MOTORS; i++) {
        ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit[i], &chan_b_config[i], &pcnt_chan_b[i]));
    }
    
    // TODO: deallocate pcnt channels when not in use

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    for (int i = 0; i < NUM_MOTORS; i++){
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a[i], PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a[i], PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b[i], PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b[i], PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    }
    
    pcnt_event_callbacks_t cbs[NUM_MOTORS] = {
        {.on_reach = example_pcnt_on_reach,},
        {.on_reach = example_pcnt_on_reach,},
        {.on_reach = example_pcnt_on_reach,},
        {.on_reach = example_pcnt_on_reach,}
    };

    QueueHandle_t queue = xQueueCreate(10, sizeof(int));
    for (int i = 0; i < NUM_MOTORS; i++){
        ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit[i], &cbs[i], queue));
        ESP_LOGI(TAG, "enable pcnt unit");
        ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit[i]));
        ESP_LOGI(TAG, "clear pcnt unit");
        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit[i]));
        ESP_LOGI(TAG, "start pcnt unit");
        ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit[i]));
    }
    
#if CONFIG_EXAMPLE_WAKE_UP_LIGHT_SLEEP
    // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
    ESP_ERROR_CHECK(gpio_wakeup_enable(EXAMPLE_EC11_GPIO_A, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    ESP_ERROR_CHECK(esp_light_sleep_start());
#endif

    // Report counter value
    int pulse_count = 0;
    while (1) {
        // if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(1000))) {
        //     ESP_LOGI(TAG, "Watch point event, count: %d", event_count);
        // } else {
            ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit[0], &pulse_count));
            ESP_LOGI(TAG, "Pulse count: %d", pulse_count);
        //}
    }
}
