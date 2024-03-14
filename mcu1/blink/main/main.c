// #include <unistd.h>
#include <stdio.h>
#include "mcu_m.h"
#include "mcu_e.h"

void app_main(void)
{
    printf("starting main\n");
    // test_drive_alternating();
    test_limit();
    test_all_stepper();
    test_z_stepper();
    printf("done main\n");
}
