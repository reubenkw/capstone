#include <unistd.h>

#include "driver/gpio.h"
#include "mc_drive.h"
#include "spi.h"
#include <sys/time.h>   

void init_dc_mc() {
    printf("starting init_dc_mc");
    
    gpio_config_t io_conf_motor = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = GPIO_DRIVE_MOTOR_PIN_SEL, 
        .pull_down_en = 1,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf_motor);


    printf("done init_dc_mc");
}

void set_wheel_directions(bool isFwd) {
    if (isFwd) {
        gpio_set_level(DIR_1, 1);
        gpio_set_level(DIR_2, 0); 
    } else {
        gpio_set_level(DIR_1, 0);
        gpio_set_level(DIR_2, 1);
    }  
}

void stop() {
    gpio_set_level(DIR_1, 0);
    gpio_set_level(DIR_2, 0);
}

#define DRIVE_PWM_DELAY 100
// only positive numbers should be passed in
void pwm(double fl, double fr, double bl, double br) {
    if (fl < 0 || fr < 0 || bl < 0 || br < 0){
        return;
    }
    gpio_set_level(PWM_FL, 1);
    gpio_set_level(PWM_FR, 1);
    gpio_set_level(PWM_BL, 1);
    gpio_set_level(PWM_BR, 1);
    bool fl_stop = false, fr_stop = false, bl_stop = false, br_stop = false;
    for (int i = 0; i <= 100; i++){
        if (!fl_stop && i > fl * 100){
            gpio_set_level(PWM_FL, 0); 
            fl_stop = true;
            printf("stopped fl at i = %d\n", i);
        }
        if (!fr_stop && i > fr * 100){
            gpio_set_level(PWM_FR, 0); 
            fr_stop = true;
            printf("stopped fr at i = %d\n", i);
        }
        if (!bl_stop && i > bl * 100){
            gpio_set_level(PWM_BL, 0); 
            bl_stop = true;
            printf("stopped bl at i = %d\n", i);
        }
        if (!br_stop && i > br * 100){
            gpio_set_level(PWM_BR, 0); 
            br_stop = true;
            printf("stopped br at i = %d\n", i);
        }
        usleep(DRIVE_PWM_DELAY);
    }   
}

// negative: backwards, 0: none, positive: forwards
// motors in order: 1, 2, 3, 4
void drive_wheels(double fl, double fr, double bl, double br, double sec) {
    struct timeval t1, t2;
    double elapsedTime = 0;
    // start timer
    gettimeofday(&t1, NULL);

    if (fl > 0 && fr > 0 && bl > 0 && br < 0){ // if all directions are pos go fwd
        set_wheel_directions(true);
        while (elapsedTime < sec){
            pwm(fl, fr, bl, br);
            gettimeofday(&t2, NULL);
            elapsedTime = (t2.tv_usec - t1.tv_usec) / 1000000.0;
            printf("elapsed time: %0.2f\n", elapsedTime);
        }
        stop();

    } else if (fl < 0 && fr < 0 && bl < 0 && br < 0) { // if all directions are neg go bkwd
        set_wheel_directions(false);
        while (elapsedTime < sec){
            pwm(-fl, -fr, -bl, -br);
            gettimeofday(&t2, NULL);
            elapsedTime = (t2.tv_usec - t1.tv_usec) / 1000000.0;
            printf("elapsed time: %0.2f\n", elapsedTime);
        }
        stop();
    } else {
        stop();
    }
}
