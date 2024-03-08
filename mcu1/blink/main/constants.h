#define DRIVE_MC 0x1    // must be same as comm.h
#define SERVO_MC 0x2    // must be same as comm.h
#define LIMIT 0x03      // must be same as comm.h
#define STATUS 0x4      // must be same as comm.h

#define ADDR_INDEX 1
#define REG_INDEX 2
#define COMMAND_INDEX 3
#define DATA_INDEX 4 

#define WRITE_CMD 1
#define READ_CMD 2

#define DRIVE_MC_ADDR 0x1000
#define SERVO_MC_ADDR 0x2000

#define PWM_1 0x13
#define PWM_2 0x14
#define PWM_3 0x15
#define PWM_4 0x16
#define MC_STAT_REG 0x00

#define STAT_OK 0x0

#define SPI_TX_ERR 0x70
#define SPI_RX_ERR 0x80

#define MC_STAT_OK 0x0
#define DRIVE_MC_ERR 0x42
#define SERVO_MC_ERR 0x24

// buck and boost
#define GPIO_BUCK 21
#define GPIO_BOOST 14

// stepper motors
// sets to chopping mode (from default continuous)
#define PWM_CTRL_1 0x0B // HB8_PWM HB7_PWM HB6_PWM HB5_PWM HB4_PWM HB3_PWM HB2_PWM HB1_PWM
#define PWM_CTRL_2 0x0C // PWM_CH4_DIS PWM_CH3_DIS PWM_CH2_DIS PWM_CH1_DIS HB12_PWM HB11_PWM HB10_PWM HB9_PWM 

// 
