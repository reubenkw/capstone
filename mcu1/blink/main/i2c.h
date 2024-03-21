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
  S_M_PROCESSING_CMD,
  S_M_ACTION_COMPLETE,
} mcu_m_status_t;

typedef enum {
  CMD_M_WRITE_STATUS,
  CMD_M_FWD,  // 5 extra data bytes: uint8_t pwm speed + 4 byte float for time (seconds)
  CMD_M_BKWD, // 5 extra data bytes: uint8_t pwm speed + 4 byte float for time (seconds)
} jetson_2_mcu_m_commands_t;

typedef enum {
  S_E_PROCESSING_CMD,
  S_E_ACTION_COMPLETE,
  S_E_ACTION_ENDED_W_LIMIT,
} mcu_e_status_t;

typedef enum {
    CMD_E_WRITE_STATUS,
    CMD_E_MOVE_AXIS,  // 3 extra data bytes
    CMD_E_RESET,
    CMD_E_POLLINATE,
} jetson_2_mcu_e_commands_t;

void init_i2c_jetson_mcu_m();
void init_i2c_jetson_mcu_e();
void test_i2c_write(uint mcu_address);
void test_i2c_read(uint mcu_address);

void i2c_write_jetson(mcu_e_status_t status);
float uint8_array_to_float(uint8_t *array);

#endif //h
