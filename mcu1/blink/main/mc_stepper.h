#ifndef MC_STEPPER
#define MC_STEPPER

#define GPIO_DIR_Z 4
#define GPIO_PULSE_Z 5
#define GPIO_DIR_Y 6
#define GPIO_PULSE_Y 7
#define GPIO_DIR_X 8
#define GPIO_PULSE_X 9

#define GPIO_MOTOR_PIN_SEL  ( \
    (1ULL<<GPIO_DIR_Z) | (1ULL<<GPIO_PULSE_Z) | \
    (1ULL<<GPIO_DIR_Y) | (1ULL<<GPIO_PULSE_Y) | \
    (1ULL<<GPIO_DIR_X) | (1ULL<<GPIO_PULSE_X)  \
)

// limit switch dimensions
#define LIMIT_X_MIN_DIST 0
#define LIMIT_X_MAX_DIST 680
#define LIMIT_Y_MIN_DIST 0
#define LIMIT_Y_MAX_DIST 340
#define LIMIT_Z_DIST 748

void init_stepper_mc();

#endif // h
