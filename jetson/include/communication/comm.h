#ifndef COMM_H
#define COMM_H

#include <cstdint>

#define MCU_1 0x01
#define MCU_ENCODER 0x10

#define DRIVE_MC 0x1
#define SERVO_MC 0x2

enum DriveMotor { frontLeft, backLeft, frontRight, backRight };

namespace Arm {
	enum ServoMotor { x, y, z };
};
namespace roboPos {
	enum robotPosition { x, y, theta};
};


int open_i2c();
void read_i2c(int file, uint8_t mcu_addr, uint8_t * data, uint8_t len);
void write_i2c(int file, uint8_t mcu_addr, uint16_t data);

#endif //h