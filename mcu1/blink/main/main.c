#include <unistd.h>
#include <stdio.h>
#include "mcu_m.h"
#include "mcu_e.h"
#include "i2c.h"

#include "spi.h"

void app_main(void)
{
    usleep(10000000);
    printf("starting main\n");
    // test_drive_full();
    // test_drive_alternating();
    // test_limit();
    // test_all_stepper();
    // test_z_stepper();
    init_spi();
    while(1) {
        printf("doing nothing. will always do nothing.\n");
        read_spi(spi_mc_dc_handle, 0);
        usleep(10000000);
    }
    printf("done main\n");
}
