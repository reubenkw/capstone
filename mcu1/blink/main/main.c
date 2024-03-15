#include <unistd.h>
#include <stdio.h>
#include "mcu_m.h"
#include "mcu_e.h"
#include "i2c.h"

void app_main(void)
{
    usleep(10000000);
    printf("starting main\n");
    test_drive_alternating();
    // test_limit();
    // test_all_stepper();
    // test_z_stepper();
    // test_drive_alternating();
    // test_i2c_write();
    printf("done main\n");
}
