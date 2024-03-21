#include <unistd.h>

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "hal/ledc_types.h"
#include "driver/gpio.h"
#include "mc_drive.h"
#include "spi.h"
#include <sys/time.h>   

void init_dc_mc() {
    printf("starting init_dc_mc");

    // set direction to output
    gpio_config_t io_conf_motor = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_DRIVE_MOTOR_PIN_SEL, 
        .pull_down_en = 1,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf_motor);
    
    // Configure PWM timer settings
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT, // Adjust as needed
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000 // PWM frequency (in Hz)
    };
    ledc_timer_config(&timer_conf);

    // Configure PWM channels
    // fl wheel
    ledc_channel_config_t pwm_channel_conf1 = {
        .gpio_num = PWM_FL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0 // Initial duty cycle (0-255 for 8-bit resolution)
    };
    ledc_channel_config(&pwm_channel_conf1);

    // fr wheel
    ledc_channel_config_t pwm_channel_conf2 = {
        .gpio_num = PWM_FR,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0 // Initial duty cycle (0-255 for 8-bit resolution)
    };
    ledc_channel_config(&pwm_channel_conf2);

    // bl wheel
    ledc_channel_config_t pwm_channel_conf3 = {
        .gpio_num = PWM_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0 // Initial duty cycle (0-255 for 8-bit resolution)
    };
    ledc_channel_config(&pwm_channel_conf3);

    // br wheel
    ledc_channel_config_t pwm_channel_conf4 = {
        .gpio_num = PWM_BR,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_3,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0 // Initial duty cycle (0-255 for 8-bit resolution)
    };
    ledc_channel_config(&pwm_channel_conf4);

    printf("done init_dc_mc");
}

void set_wheel_directions(bool isFwd) {
    if (isFwd) {
        gpio_set_level(DIR_1, 0);
        gpio_set_level(DIR_2, 1);
    } else {
        gpio_set_level(DIR_1, 1);
        gpio_set_level(DIR_2, 0); 
    }  
}

void drive_stop() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}

void drive(uint8_t fl, uint8_t fr, uint8_t bl, uint8_t br, bool fwd, double sec) {
    printf("drive start\n");
    set_wheel_directions(fwd);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, fl);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, fr);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, bl);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, br);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);

    sleep(sec);
    drive_stop();
    printf("drive done\n");
}
