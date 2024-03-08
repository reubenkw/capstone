#include "comm.h"
#include "log.h"

extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

#include <fcntl.h>    
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdio.h>

int open_i2c() {
    int file = open("/dev/i2c-1", O_RDWR);
    if (file < 0) {
        log(std::string("ERROR: failed to open i2c bus"));
    }
    return file;
}

#define READ 2
#define WRITE 1

void read_i2c(int file, uint8_t mcu_addr, uint8_t * data, uint8_t len) {
    int ioctl_val = ioctl(file, I2C_SLAVE, 0x42);
    log(std::string("INFO: ioctl_val: "));
    log(std::to_string(ioctl_val));
    if ( ioctl_val < 0) {
	    log(std::string("ERROR: failed to read from i2c bus address: "));
        return;
    }

    i2c_smbus_read_i2c_block_data(file, READ, len, data);
    log(std::string("INFO: read from i2c bus address: ") + std::to_string(mcu_addr));
    log(std::string("INFO: data: "));

    std::string dataString = std::string("(uint8_t decimal) ");
    for (int i = 0; i < len; i++){
        dataString = dataString + std::to_string(data[i]) + std::string(", ");
    }

    debug_log(std::string(dataString));
}

void write_i2c(int file, uint8_t mcu_addr, uint8_t * data, uint8_t len){
    if (ioctl(file, I2C_SLAVE, mcu_addr) < 0) {
        log(std::string("ERROR: failed to write to i2c bus address:" + mcu_addr));
    }
    int ret = i2c_smbus_write_i2c_block_data(file, WRITE, len, data);
    std::string dataString = std::string("(uint8_t decimal) ");
    for (int i = 0; i < len; i++){
        dataString = dataString + std::to_string(data[i]) + std::string(", ");
    }
    if (ret < 0) {
        log( std::string("ERROR: failed to write to i2c bus address: ") + std::to_string(mcu_addr) 
            + std::string(" cmd: ") + std::to_string(WRITE) 
            + std::string(" data: ") + dataString);
    } else {
        debug_log( std::string("ERROR: failed to write to i2c bus address: ") + std::to_string(mcu_addr) 
            + std::string(" cmd: ") + std::to_string(WRITE) 
            + std::string(" data: ") + dataString);
    }
}
