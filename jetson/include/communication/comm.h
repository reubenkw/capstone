#ifndef COMM_H
#define COMM_H

#include <cstdint>

#define MCU_M 0x42
#define MCU_E 0x10

#define DRIVE_MC 0x1
#define SERVO_MC 0x2
#define LIMIT_SWITCHES 0x3

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

int open_i2c();
void read_i2c(int file, uint8_t mcu_addr, uint8_t command, uint8_t * data, uint8_t len);
void write_i2c(int file, uint8_t mcu_addr, uint8_t command, uint8_t * data, uint8_t len);

#endif //h
