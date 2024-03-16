#include "driver/gpio.h"

#include "mc_stepper.h"

void init_stepper_mc() {
    gpio_config_t stepper_io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_MOTOR_PIN_SEL,
        .pull_down_en = 1,
        .pull_up_en = 0,
    };
    gpio_config(&stepper_io_conf);
}
