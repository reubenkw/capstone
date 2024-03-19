#include <unistd.h>
#include <stdio.h>
#include "mcu_m.h"
#include "mcu_e.h"

void app_main(void)
{
    usleep(5000000);
    printf("starting main\n");

    // test_stepper_positioning();
    // test_drive_full();
    // test_drive_alternating();
    // test_limit();
    // test_all_stepper();
    test_z_stepper();
    // test_i2c_stepper_interface();
    // test_i2c_write(MCU_E_ADDRESS);
    // test_all_stepper();
    // test_i2c_stepper_interface();
    // test_motor_go_to(260, 210, 550);
    printf("done main\n");
    while(1) {
        usleep(10000000);
    }
}
