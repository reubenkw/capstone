#include "comm.h"
#include "log.h"

extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

#include <fcntl.h>    
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>

int open_i2c() {
    int file = open("/dev/i2c-0", O_RDWR);
    if (file < 0) {
        log(std::string("ERROR: failed to open i2c bus"));
    }
    return file;
}

#define READ 2
#define WRITE 1

void read_i2c(int file, uint8_t mcu_addr, uint8_t * data, uint8_t len) {
    if (ioctl(file, I2C_SLAVE, mcu_addr) < 0) {
        log(std::string("ERROR: failed to read from i2c bus address:" + mcu_addr));
    }
    i2c_smbus_read_i2c_block_data(file, READ, len, data);
    log(std::string("INFO: read from i2c bus address:" + mcu_addr) + std::string("data: "));

    for (int i = 0; i < len; i++){
        log(std::to_string(data[i]));
    }
}

void write_i2c(int file, uint8_t mcu_addr, uint16_t data){
    if (ioctl(file, I2C_SLAVE, mcu_addr) < 0) {
        log(std::string("ERROR: failed to write to i2c bus address:" + mcu_addr));
    }
    int ret = i2c_smbus_write_word_data(file, WRITE, data);
    if (ret < 0) {
        log(  std::string("ERROR: failed to write to i2c bus address:" + mcu_addr) 
            + std::string("cmd: " + WRITE) 
            + std::string("data: " + data));
    } else {
        log(  std::string("INFO: wrote to i2c bus address:" + mcu_addr) 
            + std::string("cmd: " + WRITE) 
            + std::string("data: " + data));
    }
}

#define MAX_BUF 64

void initialize_gpio(unsigned int gpio){
    int fd;
	char buf[MAX_BUF];
 
    // export
	fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
        log(std::string("gpio/export failed"));
	}
 
	int len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);

    // set direction
    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR  "/gpio%d/direction", gpio);
 
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/direction");
        log(std::string("gpio/direction failed"));
	}
	write(fd, "in", 3);
    close(fd);

    // set edge
    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/edge", gpio);
 
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-edge");
        log(std::string("gpio/set-edge failed"));
	}
 
	write(fd, "rising", strlen("rising") + 1); 
	close(fd);
}

int read_gpio(unsigned int gpio){
    int fd;
	char buf[MAX_BUF];
	char ch;

    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
 
	fd = open(buf, O_RDONLY);
	if (fd < 0) {
		perror("gpio/get-value");
        log(std::string("gpio/get-value failed"));
		return fd;
	}
 
	read(fd, &ch, 1);
    close(fd);
	if (ch != '0') {
		return 1;
	} else {
		return 0;
	}
}

void write_gpio(unsigned int gpio, unsigned int val){
    int fd;
	char buf[MAX_BUF];
 
	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
 
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-value");
        log(std::string("gpio/set-value failed"));
	}
 
	if (val)
		write(fd, "1", 2);
	else
		write(fd, "0", 2);
 
	close(fd);
}