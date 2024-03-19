#include <unistd.h>
#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>

#include "mcu_e.h"
#include "i2c.h"
#include "mcu_gpio.h"
#include "mc_stepper.h"

// low pass filter bs
int gpio_read(uint8_t gpio_num){
    for(int i = 0; i < 10; i++){
        if (gpio_get_level(gpio_num)==0){
            return 0;
        }
    }
    return 1;
}


void test_limit() {
    init_limit_gpio();
    init_stepper_mc();
    init_i2c_jetson_mcu_e();

    gpio_set_level(6, 1);

    while(true){
        printf("Limit Switch Val: %d, %d, %d, %d, %d\n", 
        gpio_get_level(LIMIT_X_MIN),
        gpio_get_level(LIMIT_X_MAX),
        gpio_get_level(LIMIT_Y_MIN),
        gpio_get_level(LIMIT_Y_MAX),
        gpio_get_level(LIMIT_Z));
        
    }
}

void test_all_stepper() {
    init_limit_gpio();
    init_stepper_mc();

    const uint action_delay = 1 * 1000000;
    const uint step_delay = 3000;
    const uint z_step_delay = 1000;
    const uint z_dropdown = 3000;

    // positive y
    printf("y 1\n");
    gpio_set_level(GPIO_DIR_Y, 0); 
    while(gpio_read(LIMIT_Y_MAX)==0) {
        gpio_set_level(GPIO_PULSE_Y, 1);
        usleep(step_delay);
        gpio_set_level(GPIO_PULSE_Y, 0);
        usleep(step_delay);  
    }

    usleep(action_delay);

    // negative y
    int i = 0;
    printf("y 0\n");
    gpio_set_level(GPIO_DIR_Y, 1);
    while(gpio_read(LIMIT_Y_MIN)==0) {
        gpio_set_level(GPIO_PULSE_Y, 1);
        usleep(step_delay);
        gpio_set_level(GPIO_PULSE_Y, 0);
        usleep(step_delay); 
        i++;   
    } 

    usleep(action_delay);

    // positive x
    printf("x 1\n");
    gpio_set_level(GPIO_DIR_X, 1);
    while(gpio_read(LIMIT_X_MAX)==0) {
        gpio_set_level(GPIO_PULSE_X, 1);
        usleep(step_delay);
        gpio_set_level(GPIO_PULSE_X, 0);
        usleep(step_delay);    
    }

    usleep(action_delay);

    // negative x
    int j = 0;
    printf("x 0\n");
    gpio_set_level(GPIO_DIR_X, 0);
    while(gpio_read(LIMIT_X_MIN)==0) {
        gpio_set_level(GPIO_PULSE_X, 1);
        usleep(step_delay);
        gpio_set_level(GPIO_PULSE_X, 0);
        usleep(step_delay);   
        j++; 
    }

    printf("CALIBRATE STEPS (x,y)=(%d, %d)\n", j, i);
    usleep(action_delay);

    // reset to middle y
    printf("reset to middle\n");
    printf("reset y\n");
    gpio_set_level(GPIO_DIR_Y, 0);
    for (;i>0;i-=2) {
        gpio_set_level(GPIO_PULSE_Y, 1);
        usleep(step_delay);
        gpio_set_level(GPIO_PULSE_Y, 0);
        usleep(step_delay); 
    }

    usleep(action_delay);

    // reset to middle x
    printf("reset x\n");
    gpio_set_level(GPIO_DIR_X, 1);
    for (;j>0;j-=2) {
        gpio_set_level(GPIO_PULSE_X, 1);
        usleep(step_delay);
        gpio_set_level(GPIO_PULSE_X, 0);
        usleep(step_delay); 
    }

    usleep(action_delay);

    // up to limit
    printf("z 1\n");
    gpio_set_level(GPIO_DIR_Z, 1);
    while(gpio_read(LIMIT_Z)==0) {
        gpio_set_level(GPIO_PULSE_Z, 1);
        usleep(z_step_delay);
        gpio_set_level(GPIO_PULSE_Z, 0);
        usleep(z_step_delay);    
    } 
    
    usleep(action_delay);

    // dropdown fixed number of steps
    printf("z 0\n");
    gpio_set_level(GPIO_DIR_Z, 0);
    for(int k = 0; k < z_dropdown; k++){
        gpio_set_level(GPIO_PULSE_Z, 1);
        usleep(z_step_delay);
        gpio_set_level(GPIO_PULSE_Z, 0);
        usleep(z_step_delay);    
    }

    usleep(action_delay);

    // back up to limit
    printf("z 3\n");
    gpio_set_level(GPIO_DIR_Z, 1);
    while(gpio_read(LIMIT_Z)==0) {
        gpio_set_level(GPIO_PULSE_Z, 1);
        usleep(z_step_delay);
        gpio_set_level(GPIO_PULSE_Z, 0);
        usleep(z_step_delay);    
    }
}

void test_z_stepper() {
    init_limit_gpio();
    init_stepper_mc();

    const uint action_delay = 3 * 1000000;
    const uint z_step_delay = 500;

    // up to limit
    gpio_set_level(GPIO_DIR_Z, 1);
    while(gpio_read(LIMIT_Z)==0) {
        gpio_set_level(GPIO_PULSE_Z, 1);
        usleep(z_step_delay);
        gpio_set_level(GPIO_PULSE_Z, 0);
        usleep(z_step_delay);    
    } 
    
    usleep(action_delay);

    for (int z_dropdown = 250; z_dropdown <= 3000; z_dropdown += 250) {
        gpio_set_level(GPIO_DIR_Z, 0);
        for(int k = 0; k < z_dropdown; k++) {
            gpio_set_level(GPIO_PULSE_Z, 1);
            usleep(z_step_delay);
            gpio_set_level(GPIO_PULSE_Z, 0);
            usleep(z_step_delay);    
        }
        printf("Number of steps: %d\n", z_dropdown);
        usleep(action_delay);

        // back up to limit
        gpio_set_level(GPIO_DIR_Z, 1);
        while(gpio_read(LIMIT_Z)==0) {
            gpio_set_level(GPIO_PULSE_Z, 1);
            usleep(z_step_delay);
            gpio_set_level(GPIO_PULSE_Z, 0);
            usleep(z_step_delay);    
        }
        usleep(action_delay);
    }
}

void test_stepper_positioning() {
    init_limit_gpio();
    init_stepper_mc();

    printf("0 point: %d\n", z_dist_2_steps(0));
    printf("400 point: %d\n", z_dist_2_steps(400));

    uint action_delay = 2 * 1000000;

    usleep(action_delay);
    move_stepper(STP_X, 600 - 50);
    usleep(action_delay);
    move_stepper(STP_Y, 280 - 50);
    usleep(action_delay);
    // move_stepper(STP_Z, 50);
    // usleep(action_delay);
}

void test_motor_go_to(int x, int y, int z) {
    init_limit_gpio();
    init_stepper_mc();

    reset_xyz();

    // move_x(x);
    // move_y(y);
    // move_z(LIMIT_Z_MAX_DIST, z);
}

void test_i2c_stepper_interface() {
    printf("starting init\n");
    init_limit_gpio();
    init_stepper_mc();
    init_i2c_jetson_mcu_e();
    printf("done init\n");
    usleep(500000);

    // max data len: 1 command byte then 3 data bytes for motor move
    uint8_t rx_data[3] = {0};
    mcu_e_status_t state = S_ACTION_COMPLETE;
    i2c_write_jetson(S_ACTION_COMPLETE);
    while(true) {
        // read from jetson
        printf("waiting for command. willing to wait 7 days.\n");
        i2c_slave_read_buffer(I2C_HOST, rx_data, 1, portMAX_DELAY);
        printf("got command: %x\n", rx_data[0]);
        usleep(10000);
        switch (rx_data[0]) {
            case CMD_WRITE_STATUS:
                i2c_write_jetson(state);
                break;
            case CMD_MOVE_AXIS:
                state = S_PROCESSING_CMD;
                i2c_write_jetson(state);
                i2c_slave_read_buffer(I2C_HOST, rx_data, 3, portMAX_DELAY);

                // interpret data recieved
                uint16_t ideal_pos_10x = rx_data[1] << 8 | rx_data[2];
                printf("ideal_pos_10x: %d\n", ideal_pos_10x);
                float ideal_pos = (double) ideal_pos_10x / 10.0;
                printf("Move stepper motor: %d to position %0.2f from position %0.2f\n", 
                    rx_data[0], ideal_pos, end_effector_position[rx_data[0]]);
                
                move_stepper(rx_data[0], ideal_pos);
                printf("moving arm.\n");

                // emulate moving motors
                // printf("pretending to move motors\n.");
                // usleep(4 * 1000000);

                // TODO: send if limit switch hit
                // update status to done
                state = S_ACTION_COMPLETE;
                i2c_write_jetson(state);
                break;
            case CMD_RESET:
                state = S_PROCESSING_CMD;
                i2c_write_jetson(state);
                reset_xyz();
                printf("reset arm.\n");

                // emulate moving motors
                // printf("pretending to move motors (reset)\n.");
                // usleep(4 * 1000000);

                state = S_ACTION_COMPLETE;
                i2c_write_jetson(state);
                break;
            case CMD_POLLINATE:
                state = S_PROCESSING_CMD;
                i2c_write_jetson(state);

                pollinate();
                printf("pollinating.\n");

                state = S_ACTION_COMPLETE;
                i2c_write_jetson(state);
        }
        usleep(5000);
    }
}
