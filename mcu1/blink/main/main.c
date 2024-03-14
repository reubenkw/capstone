// #include <unistd.h>
#include <stdio.h>
#include "mcu_m.h"
#include "mcu_e.h"

void app_main(void)
{
    printf("starting main\n");
    main_mcu_m();
    printf("done main\n");
}
