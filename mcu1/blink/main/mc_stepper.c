#include <math.h>
#include <unistd.h>
#include "driver/gpio.h"

#include "mc_stepper.h"
#include "mcu_gpio.h"
#include "i2c.h"

uint default_step_delays[3] = {X_STEP_DELAY, Y_STEP_DELAY, Z_STEP_DELAY};

void init_stepper_mc() {
    gpio_config_t stepper_io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_STEPPER_MOTOR_PIN_SEL,
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

void step(uint8_t pin, uint step_delay) {
    gpio_set_level(pin, 1);
    usleep(step_delay);
    gpio_set_level(pin, 0);
    usleep(step_delay);  
}

float end_effector_position[3] = {0, 0, LIMIT_Z_MAX_DIST};
// uses limit switches but no bounding on input. returns the absolute position moved to.
float move_x(float delta, uint step_delay) {
    int steps = (int) delta / X_DIST_PER_STEP;
    if (steps > 0) { // move forward
        gpio_set_level(GPIO_DIR_X, 1);
        for(; steps > 0; steps--) {
            if (gpio_get_level(LIMIT_X_MAX) == 1){
                printf("hit LIMIT_X_MAX switch");
                return LIMIT_X_MAX_DIST;
            }
            step(GPIO_PULSE_X, step_delay);
        }
    } else {
        gpio_set_level(GPIO_DIR_X, 0);
        for(; steps < 0; steps++) {
            if (gpio_get_level(LIMIT_X_MIN) == 1){
                printf("hit LIMIT_X_MIN switch");
                return LIMIT_X_MIN_DIST;
            }
            step(GPIO_PULSE_X, step_delay);
        }
    }
    return end_effector_position[STP_X] + delta;
}

float move_y(float delta, uint step_delay) {
    int steps = (int) delta / Y_DIST_PER_STEP;
    if (steps > 0) { // move forward
        gpio_set_level(GPIO_DIR_Y, 0);
        for(; steps > 0; steps--) {
            if (gpio_get_level(LIMIT_Y_MAX) == 1){
                printf("hit LIMIT_Y_MAX switch");
                return LIMIT_Y_MAX_DIST;
            }
            step(GPIO_PULSE_Y, step_delay);
        }
    } else {
        gpio_set_level(GPIO_DIR_Y, 1);
        for(; steps < 0; steps++) {
            if (gpio_get_level(LIMIT_Y_MIN) == 1){
                printf("hit LIMIT_Y_MIN switch");
                return LIMIT_Y_MIN_DIST;
            }
            step(GPIO_PULSE_Y, step_delay);
        }
    }
    return end_effector_position[STP_Y] + delta;
}

// non linear
int move_z(uint crnt, uint desired, uint step_delay) {
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
            step(GPIO_PULSE_Z, step_delay);
        }
    } else {
        gpio_set_level(GPIO_DIR_Z, 0);
        for(; steps < 0; steps++) {
            step(GPIO_PULSE_Z, step_delay);
        }
    }
    return desired;
}

void reset_xyz() {
    move_z(0, 9999, Z_STEP_DELAY); // current could be unknown, just move up to limit
    move_x(-9999, X_STEP_DELAY);
    move_y(-9999, Y_STEP_DELAY);
    end_effector_position[STP_X] = 0;
    end_effector_position[STP_Y] = 0;
    end_effector_position[STP_Z] = LIMIT_Z_MAX_DIST;
}

// return false if hit limit
bool move_stepper(uint8_t motor, float ideal_pos, int step_delay) {
    int final = 0;
    if (motor == STP_X) {
        float delta = ideal_pos - end_effector_position[motor];
        final = move_x(delta, step_delay);
    } else if (motor == STP_Y) {
        float delta = ideal_pos - end_effector_position[motor];
        final = move_y(delta, step_delay);
    } else if (motor == STP_Z) {
        final = move_z(end_effector_position[STP_Z], ideal_pos, step_delay);
    }
    end_effector_position[motor] = final;
    printf("final - ideal: %0.2f\n", final-ideal_pos);
    return fabs(final-ideal_pos) < 1;
}

void pollinate() {
    const uint delta = 30;
    const uint n = 2;
    const uint action_delay = 100000;
    const uint step_delay = 10000;

    const uint x_init = end_effector_position[STP_X];
    for (int i=0; i<n; i++) {
        move_stepper(STP_X, x_init-delta, step_delay);
        usleep(action_delay);
        move_stepper(STP_X, x_init+delta, step_delay);
        usleep(action_delay);
    }
    move_stepper(STP_X, x_init, step_delay);

    const uint y_init = end_effector_position[STP_Y];
    for (int i=0; i<n; i++) {
        move_stepper(STP_Y, y_init-delta, step_delay);
        usleep(action_delay);
        move_stepper(STP_Y, y_init+delta, step_delay);
        usleep(action_delay);
    }
    move_stepper(STP_Y, y_init, step_delay);

    // maybe a spiral?
    move_stepper(STP_X, x_init+delta/4, step_delay);
    usleep(action_delay);
    move_stepper(STP_Y, y_init+delta/4, step_delay);
    usleep(action_delay);
    move_stepper(STP_X, x_init-delta/4, step_delay);
    usleep(action_delay);
    move_stepper(STP_Y, y_init-delta/4, step_delay);
    usleep(action_delay);
    move_stepper(STP_X, x_init+delta/2, step_delay);
    usleep(action_delay);
    move_stepper(STP_Y, y_init+delta/2, step_delay);
    usleep(action_delay);
    move_stepper(STP_X, x_init-delta/2, step_delay);
    usleep(action_delay);
    move_stepper(STP_Y, y_init-delta/2, step_delay);

    // reset
    usleep(action_delay);
    move_stepper(STP_X, x_init, step_delay);
    usleep(action_delay);
    move_stepper(STP_Y, y_init, step_delay);
    usleep(action_delay);
}

void pollinate_v2(){
    const int x_delta = 30;
    const int y_delta = 15;
    const uint action_delay = 50000;
    const uint step_delay = 5000;

    const int x_init = end_effector_position[STP_X] - 20; // seems offset
    const int y_init = end_effector_position[STP_Y];
    
    move_stepper(STP_X, x_init-x_delta, step_delay);
    usleep(action_delay);
    move_stepper(STP_Y, y_init-y_delta, step_delay);
    usleep(action_delay);
    move_stepper(STP_X, x_init+x_delta, step_delay);
    usleep(action_delay);
    move_stepper(STP_Y, y_init, step_delay);
    usleep(action_delay);
    move_stepper(STP_X, x_init-x_delta, step_delay);
    usleep(action_delay);
    move_stepper(STP_Y, y_init+y_delta, step_delay);
    usleep(action_delay);
    move_stepper(STP_X, x_init+x_delta, step_delay);
    usleep(action_delay); 
}