#include "gpio.h"

uint8_t limit_pins[5] = {LIMIT_X_MIN, LIMIT_X_MAX, LIMIT_Y_MIN, LIMIT_Y_MAX, LIMIT_Z};
static led_strip_handle_t led_strip;

void dev_led_set_color(int r, int g, int b) {
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

void init_dev_led() {
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

void initialize_led() {
    // LED
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_LED_PIN_SEL,
        .pull_down_en = 1,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
}

void init_boost() {
    // boost high to disable
    gpio_config_t io_conf_output = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1 << GPIO_BOOST),
        .pull_down_en = 1,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf_output);

    // set boost low (turn on)
    gpio_set_level(GPIO_BOOST, 1);
}

void init_limit_gpio() {
    //zero-initialize the config structure.
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,     // disable interrupt
        .mode = GPIO_MODE_INPUT,            //set as input mode
        .pin_bit_mask = GPIO_LIMIT_PIN_SEL, //bit mask of the pins that you want to set,e.g.GPIO18/19
        .pull_down_en = 1,                  //disable pull-down mode
        .pull_up_en = 0,                    //disable pull-up mode
    };
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

void test_limit() {
    init_limit_gpio();

    while(true){
        if (gpio_get_level(LIMIT_X_MIN)){
            printf("xmin\n");
        }
        if (gpio_get_level(LIMIT_X_MAX)){
            printf("xmax\n");
        }
        if (gpio_get_level(LIMIT_Y_MIN)){
            printf("ymin\n");
        }
        if (gpio_get_level(LIMIT_Y_MAX)){
            printf("ymax\n");
        }
        if (gpio_get_level(LIMIT_Z)){
            printf("z\n");
        }
    }
}
