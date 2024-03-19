#include <math.h>
#include <unistd.h>
#include "driver/gpio.h"

#include "mc_stepper.h"
#include "mcu_gpio.h"
#include "i2c.h"

void init_stepper_mc() {
    gpio_config_t stepper_io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_MOTOR_PIN_SEL,
        .pull_down_en = 1,
        .pull_up_en = 0,
    };
    gpio_config(&stepper_io_conf);
    usleep(10000);
    // gpio_set_level(GPIO_STP_ENABLE, 1);
}

uint z_dist_2_steps(uint tip_z) {
    uint calibrated_point = tip_z + LEN_PAINT_BRUSH;
    return Z_POLY_COEF_2 * pow(calibrated_point, 2) + Z_POLY_COEF_1 * calibrated_point + Z_POLY_COEF_0;
}

void step(uint8_t pin) {
    uint step_delay = 1000;
    gpio_set_level(pin, 1);
    usleep(step_delay);
    gpio_set_level(pin, 0);
    usleep(step_delay);  
}

float end_effector_position[3] = {0, 0, LIMIT_Z_MAX_DIST};
// uses limit switches but no bounding on input. returns the absolute position moved to.
int move_x(int delta) {
    int steps = delta / X_DIST_PER_STEP;
    if (steps > 0) { // move forward
        gpio_set_level(GPIO_DIR_X, 1);
        for(; steps > 0; steps--) {
            if (gpio_get_level(LIMIT_X_MAX) == 1){
                printf("hit LIMIT_X_MAX switch");
                return LIMIT_X_MAX_DIST;
            }
            step(GPIO_PULSE_X);
        }
    } else {
        gpio_set_level(GPIO_DIR_X, 0);
        for(; steps < 0; steps++) {
            if (gpio_get_level(LIMIT_X_MIN) == 1){
                printf("hit LIMIT_X_MIN switch");
                return LIMIT_X_MIN_DIST;
            }
            step(GPIO_PULSE_X);
        }
    }
    return end_effector_position[STP_X] + delta;
}

int move_y(int delta) {
    int steps = delta / Y_DIST_PER_STEP;
    if (steps > 0) { // move forward
        gpio_set_level(GPIO_DIR_Y, 0);
        for(; steps > 0; steps--) {
            if (gpio_get_level(LIMIT_Y_MAX) == 1){
                printf("hit LIMIT_Y_MAX switch");
                return LIMIT_Y_MAX_DIST;
            }
            step(GPIO_PULSE_Y);
        }
    } else {
        gpio_set_level(GPIO_DIR_Y, 1);
        for(; steps < 0; steps++) {
            if (gpio_get_level(LIMIT_Y_MIN) == 1){
                printf("hit LIMIT_Y_MIN switch");
                return LIMIT_Y_MIN_DIST;
            }
            step(GPIO_PULSE_Y);
        }
    }
    return end_effector_position[STP_Y] + delta;
}

// non linear
int move_z(uint crnt, uint desired) {
    if (desired < LIMIT_Z_MIN_DIST) {
        printf("ERROR: desired z location not reachable, saturating z to LIMIT_Z_MIN_DIST\n");
        desired = LIMIT_Z_MIN_DIST;
    }
    int steps = z_dist_2_steps(crnt) - z_dist_2_steps(desired);
    if (steps > 0) { // move up
        gpio_set_level(GPIO_DIR_Z, 1);
        for(; steps > 0; steps--) {
            if (gpio_get_level(LIMIT_Z) == 1){
                printf("hit LIMIT_Z switch");
                return LIMIT_Z_MAX_DIST;
            }
            step(GPIO_PULSE_Z);
        }
    } else {
        gpio_set_level(GPIO_DIR_Z, 0);
        for(; steps < 0; steps++) {
            step(GPIO_PULSE_Z);
        }
    }
    return desired;
}

void reset_xyz() {
    move_z(0, 9999); // current could be unknown, just move up to limit
    move_x(-9999);
    move_y(-9999);
    end_effector_position[STP_X] = 0;
    end_effector_position[STP_Y] = 0;
    end_effector_position[STP_Z] = LIMIT_Z_MAX_DIST;
}

void move_stepper(uint8_t motor, float ideal_pos) {
    int final = 0;
    if (motor == STP_X) {
        float delta = ideal_pos - end_effector_position[motor];
        final = move_x(delta);
    } else if (motor == STP_Y) {
        float delta = ideal_pos - end_effector_position[motor];
        final = move_y(delta);
    } else if (motor == STP_Z) {
        final = move_z(end_effector_position[STP_Z], ideal_pos);
    }
    end_effector_position[motor] = final;
}
