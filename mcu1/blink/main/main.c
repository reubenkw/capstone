#include <unistd.h>
#include <stdio.h>
#include "mcu_m.h"
#include "mcu_e.h"
#include "i2c.h"

#include "spi.h"

void app_main(void)
{
    printf("starting main\n");
    usleep(5000000);
    // test_drive_full();
    // test_drive_alternating();
    // test_limit();
    test_all_stepper();
    // test_z_stepper();
    // test_i2c_stepper_interface();
    printf("done main\n");
}
