#include <unistd.h>
#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>

#include "mcu_e.h"
#include "i2c.h"
#include "mcu_gpio.h"
#include "mc_stepper.h"

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

// low pass filter bs
int gpio_read(uint8_t gpio_num){
    for(int i = 0; i < 10; i++){
        if (gpio_get_level(gpio_num)==0){
            return 0;
        }
    }
    return 1;
}

void test_all_stepper(){
    init_limit_gpio();
    init_boost();
    init_stepper_mc();

    const uint action_delay = 1 * 1000000;
    const uint step_delay = 1000;
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
