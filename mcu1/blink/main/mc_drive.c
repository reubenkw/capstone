#include <unistd.h>

#include "driver/gpio.h"
#include "mc_drive.h"
#include "spi.h"

void config_motor_fault() {
    gpio_config_t io_conf_motor = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL<<FAULT_DC_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf_motor);
}

void init_dc_mc() {
    printf("starting init_dc_mc");
    config_motor_fault();

    // disable OLD for all
    write_spi(spi_mc_dc_handle, 0x19, 0xFF);
    write_spi(spi_mc_dc_handle, 0x1A, 0b11001111);

    // Set OVP limit to 33 V
    uint8_t reg = read_spi(spi_mc_dc_handle, CONFIG_CTRL);
    write_spi(spi_mc_dc_handle, CONFIG_CTRL, reg | 0b10);

    // set pwm mode for ...
    write_spi(spi_mc_dc_handle, PWM_CTRL_1, 0b11000000);

    // Clear fault reg
    reg = read_spi(spi_mc_dc_handle, CONFIG_CTRL);
    write_spi(spi_mc_dc_handle, CONFIG_CTRL, reg | 0b01);

    if (gpio_get_level(FAULT_DC_GPIO) == 0){
        printf("fault\n");
        read_spi(spi_mc_dc_handle, 0);
    }

    printf("done init_dc_mc");
}

bool check_and_clear_fault() {
    if (gpio_get_level(FAULT_DC_GPIO) == 0){
        printf("fault\n");
        read_spi(spi_mc_dc_handle, 0);
        uint8_t reg = read_spi(spi_mc_dc_handle, CONFIG_CTRL);
        write_spi(spi_mc_dc_handle, CONFIG_CTRL, reg | 0b01);
        return true;
    }
    printf("no fault\n");
    read_spi(spi_mc_dc_handle, 0);
    return false;
}

// negative: backwards, 0: none, positive: forwards
// motors in order: 1, 2, 3, 4
void set_wheel_directions(int fl, int fr, int bl, int br) {
    // 0b01100000;  front right fwd
    // 0b0000110;   front left bkwd
    // 0b01100000;  back right fwd
    // 0b01100000;  back right bkwd
    uint8_t op_ctrl_1_val = 0;
    uint8_t op_ctrl_2_val = 0;

    if (fl > 0) {
        op_ctrl_1_val |= 0b0000110;
    } else if (fl < 0) {
        op_ctrl_1_val |= 0b0001001;
    }

    if (fr > 0) {
        op_ctrl_1_val |= 0b01100000;
    } else if (fr < 0) {
        op_ctrl_1_val |= 0b10010000;
    }

    if (bl > 0) {
        op_ctrl_2_val |= 0b0001001;
    } else if (bl < 0) {
        op_ctrl_2_val |= 0b0000110;
    }

    if (br > 0) {
        op_ctrl_2_val |= 0b01100000;
    } else if (br < 0) {
        op_ctrl_2_val |= 0b10010000;
    }

    write_spi(spi_mc_dc_handle, OP_CTRL_1, op_ctrl_1_val);
    write_spi(spi_mc_dc_handle, OP_CTRL_2, op_ctrl_2_val);
}

void drive_full_forward() {
    set_wheel_directions(1,1,1,1);
    write_spi(spi_mc_dc_handle, PWM_DUTY_CTRL_1, 0x7F);
}

void drive_alternate_direction() {
    const uint accel_delay = 1000;
    const uint full_speed_delay = 5 * 1000000;
    const uint stopped_delay = 2 * 1000000;
    const uint8_t min_pwm = 20;
    uint8_t speed = 0;
    while(true) {
        set_wheel_directions(1, 1, 1, 1);
        printf("speeding up forward!\n");
        for (speed=min_pwm; speed<254; speed++) {
            write_spi(spi_mc_dc_handle, PWM_DUTY_CTRL_1, speed);
            if (check_and_clear_fault()) {
                speed--;
                printf("%d\n", speed);
            }
            usleep(accel_delay);
        }

        // stay at speed
        printf("full speed forward!\n");
        usleep(full_speed_delay);

        printf("slowing down!\n");
        for (speed=254; speed>10; speed--) {
            write_spi(spi_mc_dc_handle, PWM_DUTY_CTRL_1, speed);
            if (check_and_clear_fault()) {
                speed++;
                printf("%d\n", speed);
            }
            usleep(accel_delay);
        } 
        write_spi(spi_mc_dc_handle, PWM_DUTY_CTRL_1, 0);

        // stay at stopped
        printf("stopped!\n");
        usleep(stopped_delay);

        // set direction to reversed
        set_wheel_directions(-1, -1, -1, -1);
        printf("speeding up backwards!\n");
        for (speed=min_pwm; speed<254; speed++) {
            write_spi(spi_mc_dc_handle, PWM_DUTY_CTRL_1, speed);
            if (check_and_clear_fault()) {
                speed--;
                printf("%d\n", speed);
            }
            usleep(accel_delay);
        }

        // stay at speed
        printf("full speed backwards!\n");
        usleep(full_speed_delay);

        printf("slowing down!\n");
        for (speed=254; speed>10; speed--) {
            write_spi(spi_mc_dc_handle, PWM_DUTY_CTRL_1, speed);
            if (check_and_clear_fault()) {
                speed++;
                printf("%d\n", speed);
            }
            usleep(accel_delay);
        } 
        write_spi(spi_mc_dc_handle, PWM_DUTY_CTRL_1, 0);

        // stay at stopped
        printf("stopped!\n");
        usleep(stopped_delay);
    }
}
