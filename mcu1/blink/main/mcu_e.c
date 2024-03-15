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

    while(true){
        printf("Limit Switch Val: %d, %d, %d, %d, %d\n", 
        gpio_get_level(LIMIT_X_MIN),
        gpio_get_level(LIMIT_X_MAX),
        gpio_read(LIMIT_Y_MIN),
        gpio_get_level(LIMIT_Y_MAX),
        gpio_get_level(LIMIT_Z));
        
    }
}


void test_all_stepper(){
    init_limit_gpio();
    init_boost();
    init_stepper_mc();

    const uint action_delay = 5 * 1000000;
    const uint step_delay = 10000;
    const uint z_step_delay = 1000;
    const uint z_dropdown = 2000;

    // positive y
    gpio_set_level(GPIO_DIR_Y, 0); 
    while(gpio_read(LIMIT_Y_MAX)==0) {
        printf("y 1\n");
        gpio_set_level(GPIO_PULSE_Y, 1);
        usleep(step_delay);
        gpio_set_level(GPIO_PULSE_Y, 0);
        usleep(step_delay);  
    }

    usleep(action_delay);

    // negative y
    int i = 0;
    gpio_set_level(GPIO_DIR_Y, 1);
    while(gpio_read(LIMIT_Y_MIN)==0) {
        printf("y 0\n");
        gpio_set_level(GPIO_PULSE_Y, 1);
        usleep(step_delay);
        gpio_set_level(GPIO_PULSE_Y, 0);
        usleep(step_delay); 
        i++;   
    } 

    usleep(action_delay);

    // positive x
    gpio_set_level(GPIO_DIR_X, 1);
    while(gpio_read(LIMIT_X_MAX)==0) {
        printf("x 1\n");
        gpio_set_level(GPIO_PULSE_X, 1);
        usleep(step_delay);
        gpio_set_level(GPIO_PULSE_X, 0);
        usleep(step_delay);    
    }

    usleep(action_delay);

    // negative x
    int j = 0;
    gpio_set_level(GPIO_DIR_X, 0);
    while(gpio_read(LIMIT_X_MIN)==0) {
        printf("x 0\n");
        gpio_set_level(GPIO_PULSE_X, 1);
        usleep(step_delay);
        gpio_set_level(GPIO_PULSE_X, 0);
        usleep(step_delay);   
        j++; 
    }

    usleep(action_delay);

    // reset to middle y
    printf("reset to middle\n");
    gpio_set_level(GPIO_DIR_Y, 0);
    for (;i>0;i-=2) {

        printf("reset y\n");
        gpio_set_level(GPIO_PULSE_Y, 1);
        usleep(step_delay);
        gpio_set_level(GPIO_PULSE_Y, 0);
        usleep(step_delay); 
    }

    usleep(action_delay);

    // reset to middle x
    gpio_set_level(GPIO_DIR_X, 1);
    for (;j>0;j-=2) {
        printf("reset x\n");
        gpio_set_level(GPIO_PULSE_X, 1);
        usleep(step_delay);
        gpio_set_level(GPIO_PULSE_X, 0);
        usleep(step_delay); 
    }

    usleep(action_delay);

    // up to limit
    gpio_set_level(GPIO_DIR_Z, 1);
    while(gpio_read(LIMIT_Z)==0) {
        printf("z 1\n");
        gpio_set_level(GPIO_PULSE_Z, 1);
        usleep(z_step_delay);
        gpio_set_level(GPIO_PULSE_Z, 0);
        usleep(z_step_delay);    
    } 
    
    usleep(action_delay);

    // dropdown fixed number of steps
    gpio_set_level(GPIO_DIR_Z, 0);
    for(int k = 0; k < z_dropdown; k++){
        printf("z 0\n");
        gpio_set_level(GPIO_PULSE_Z, 1);
        usleep(z_step_delay * 2);
        gpio_set_level(GPIO_PULSE_Z, 0);
        usleep(z_step_delay * 2);    
    }

    usleep(action_delay);

    // back up to limit
    gpio_set_level(GPIO_DIR_Z, 1);
    while(gpio_read(LIMIT_Z)==0) {
        printf("z 3\n");
        gpio_set_level(GPIO_PULSE_Z, 1);
        usleep(z_step_delay);
        gpio_set_level(GPIO_PULSE_Z, 0);
        usleep(z_step_delay);    
    }
}

void test_z_stepper() {
    init_limit_gpio();
    init_boost();
    init_stepper_mc();

    const uint action_delay = 1 * 1000000;
    const uint z_step_delay = 1000;
    const uint z_dropdown = 2000;

    // up to limit
    gpio_set_level(GPIO_DIR_Z, 1);
    while(gpio_read(LIMIT_Z)==0) {
        printf("z 1\n");
        gpio_set_level(GPIO_PULSE_Z, 1);
        usleep(z_step_delay);
        gpio_set_level(GPIO_PULSE_Z, 0);
        usleep(z_step_delay);    
    } 
    
    usleep(action_delay);

    // dropdown fixed number of steps
    gpio_set_level(GPIO_DIR_Z, 0);
    for(int k = 0; k < z_dropdown; k++){
        printf("z 0\n");
        gpio_set_level(GPIO_PULSE_Z, 1);
        usleep(z_step_delay * 2);
        gpio_set_level(GPIO_PULSE_Z, 0);
        usleep(z_step_delay * 2);    
    }

    usleep(action_delay);

    // back up to limit
    gpio_set_level(GPIO_DIR_Z, 1);
    while(gpio_read(LIMIT_Z)==0) {
        printf("z 3\n");
        gpio_set_level(GPIO_PULSE_Z, 1);
        usleep(z_step_delay);
        gpio_set_level(GPIO_PULSE_Z, 0);
        usleep(z_step_delay);    
    }
}

void step(uint8_t pin){
    uint step_delay = 1000;
    gpio_set_level(pin, 1);
    usleep(step_delay);
    gpio_set_level(pin, 0);
    usleep(step_delay);  
}

float pos[3] = {0, 0, LIMIT_Z};
bool move_stepper(uint8_t motor, float ideal_pos){
    bool fwd_dir[3] = {1,0,1};
    uint8_t dir[3] = {GPIO_DIR_X, GPIO_DIR_Y, GPIO_DIR_Z};
    uint8_t pulse[3] = {GPIO_PULSE_X, GPIO_PULSE_Y, GPIO_PULSE_Z};
    uint8_t fwd_limit[3] = {LIMIT_X_MAX, LIMIT_Y_MAX, LIMIT_Z};
    uint8_t bkwd_limit[2] = {LIMIT_X_MIN, LIMIT_Y_MIN};
    uint16_t max_limits[3] = {LIMIT_X_MAX_DIST, LIMIT_Y_MAX_DIST, LIMIT_Z_DIST};
    uint16_t min_limits[3] = {0, 0, 400};

    float tol = 0.163;
    float delta = ideal_pos - pos[motor];
    if (delta > tol){ // move forward
        gpio_set_level(dir[motor], fwd_dir[motor]);
        while(delta > tol){
            if (gpio_get_level(fwd_limit[motor]) == 0){
                pos[motor] = max_limits[motor];
                printf("hit fwd limit switch: %d", motor);
                return true;
            }
            step(pulse[motor]);
            delta -= 0.163;  
        }
    } else if (delta < -tol){ // move backward
        gpio_set_level(dir[motor], !fwd_dir[motor]);
        while(delta < -tol){
            if ((motor < STP_Z && gpio_get_level(bkwd_limit[motor]) == 0) || 
                (pos[motor] < min_limits[STP_Z])){
                pos[motor] = min_limits[motor];
                printf("hit bkwd limit switch: %d", motor);
                return true;
            }
            step(pulse[motor]);
            delta += 0.163;
        }
    }
    pos[motor] = ideal_pos - delta;    
    return false;
}

void test_i2c_stepper_interface(){
    init_boost();
    init_stepper_mc();
    init_i2c_jetson();
    printf("done init\n");

    uint8_t rx_data[I2C_DATA_LENGTH + 1] = {0};
    while(true){
        // wait until jetson nano reads from mcu1 over i2c 
        // based on jetson nano command, do different tasks
        printf("waiting for command. willing to wait 7 days.\n");
        i2c_slave_read_buffer(I2C_HOST, rx_data, I2C_DATA_LENGTH + 1, portMAX_DELAY);
        dev_led_set_color(0, 50, 50);
        printf("read from jetson: %d %d %d %d \n", rx_data[1], rx_data[2], rx_data[3], rx_data[4]); 
        
        switch (rx_data[ADDR_INDEX]) {
            case STP_X:
            case STP_Y: 
            case STP_Z: {
                uint16_t ideal_pos_10x = rx_data[2] << 8 & rx_data[3];
                float ideal_pos = (double) ideal_pos_10x / 10.0;
                printf("Move stepper motor: %d to position %0.2f from position %0.2f\n", 
                    rx_data[1], ideal_pos, pos[rx_data[ADDR_INDEX]]); 
                uint8_t data[2] = {0x01, 0x00};
                if (move_stepper(rx_data[ADDR_INDEX], ideal_pos)){
                    data[0] = 2; // hit limit switch
                }
                int data_trans = i2c_slave_write_buffer(I2C_HOST, data, 2, portMAX_DELAY);
                printf("data trans: %d\n", data_trans);
                break;
            }
            case ENC:
                printf("encoder address!");
            break;
        }
    }
}
