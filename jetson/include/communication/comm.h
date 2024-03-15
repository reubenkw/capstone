#ifndef COMM_H
#define COMM_H

#include <cstdint>

#define MCU_M 0x42
#define MCU_E 0x10

#define DRIVE_MC 0x1
#define SERVO_MC 0x2
#define LIMIT_SWITCHES 0x3

#define WRITE_CMD 1
#define READ_CMD 2

int open_i2c();
void read_i2c(int file, uint8_t mcu_addr, uint8_t * data, uint8_t len);
void write_i2c(int file, uint8_t mcu_addr, uint8_t * data, uint8_t len);

#endif //h
