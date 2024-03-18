#ifndef I2C_H
#define I2C_H

#include "driver/i2c.h"

#define MCU_M_ADDRESS 0x42
#define MCU_E_ADDRESS 0x10

#define I2C_HOST I2C_NUM_0

#define STP_X 0
#define STP_Y 1
#define STP_Z 2

#define DRIVE_MC 0x1    // must be same as comm.h
#define SERVO_MC 0x2    // must be same as comm.h

typedef enum {
  S_WAITING,
  S_ACTION_COMPLETE,
  S_ACTION_ENDED_W_LIMIT,
} mcu_e_status_t;

typedef enum {
    CMD_WRITE_STATUS,
    CMD_MOVE_AXIS,  // 3 extra data bytes
    CMD_RESET,
    CMD_POLINATE,
} jetson_2_mcu_e_commands_t;

void init_i2c_jetson_mcu_m();
void init_i2c_jetson_mcu_e();
void test_i2c_write(uint mcu_address);
void test_i2c_read(uint mcu_address);

void i2c_write_jetson(mcu_e_status_t status);

#endif //h
