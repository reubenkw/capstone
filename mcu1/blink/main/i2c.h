#ifndef I2C_H
#define I2C_H

#include "driver/i2c.h"

#define I2C_HOST I2C_NUM_0
#define I2C_DATA_LENGTH 4

#define ADDR_INDEX 1
#define CMD_INDEX 2
#define REG_INDEX 3
#define DATA_INDEX 4 

#define STP_X 0
#define STP_Y 1
#define STP_Z 2
#define ENC 3

#define DRIVE_MC 0x1    // must be same as comm.h
#define SERVO_MC 0x2    // must be same as comm.h

#define STOP_CMD 0
#define GO_CMD 1
#define BKWD_CMD 2

void init_i2c_jetson();
void test_i2c_write();
void test_i2c_read();

#endif //h
