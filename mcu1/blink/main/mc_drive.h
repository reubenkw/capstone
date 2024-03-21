#ifndef MC_DRIVE_H
#define MC_DRIVE_H

#define PWM_FL 2 // pin 2 is fucked for some reason, all of them move dk why, might be electrical issue
#define PWM_FR 1
#define PWM_BL 43
#define PWM_BR 11

#define DIR_1 12
#define DIR_2 13

#define GPIO_DRIVE_MOTOR_PIN_SEL  ( \
    (1ULL<<DIR_1) | (1ULL<<DIR_2)  \
)

void init_dc_mc();
void drive_stop();
void drive(uint8_t fl, uint8_t fr, uint8_t bl, uint8_t br, bool fwd, double sec);

#endif // h
