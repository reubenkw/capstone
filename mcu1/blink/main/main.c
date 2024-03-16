#include <unistd.h>
#include <stdio.h>
#include "mcu_m.h"
#include "mcu_e.h"
#include "i2c.h"

void app_main(void)
{
    usleep(5000000);
    printf("starting main\n");
    // test_drive_full();
    // test_drive_alternating();
    // test_limit();
    // test_all_stepper();
    // test_z_stepper();
    test_i2c_stepper_interface();
    // test_i2c_read();
    printf("done main\n");
}
