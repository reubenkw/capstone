#ifndef MC_DRIVE_H
#define MC_DRIVE_H

#define DRIVE_MC_ADDR 0x1000
#define SERVO_MC_ADDR 0x2000

#define PWM_1 0x13
#define PWM_2 0x14
#define PWM_3 0x15
#define PWM_4 0x16
#define MC_STAT_REG 0x00

#define MC_STAT_OK 0x0
#define DRIVE_MC_ERR 0x42
#define SERVO_MC_ERR 0x24

// stepper motors only uses these registers to disabl PWM
#define PWM_CTRL_1 0x0B // HB8_PWM HB7_PWM HB6_PWM HB5_PWM HB4_PWM HB3_PWM HB2_PWM HB1_PWM
#define PWM_CTRL_2 0x0C // PWM_CH4_DIS PWM_CH3_DIS PWM_CH2_DIS PWM_CH1_DIS HB12_PWM HB11_PWM HB10_PWM HB9_PWM 

#define OP_CTRL_1 0x08
#define OP_CTRL_2 0x09
#define OP_CTRL_3 0x0A
// HB8_HS_EN HB8_LS_EN HB7_HS_EN HB7_LS_EN
// HB6_HS_EN HB6_LS_EN HB5_HS_EN HB5_LS_EN

void drive_full_forward();

#endif // h
