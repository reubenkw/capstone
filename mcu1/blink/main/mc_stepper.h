#ifndef MC_STEPPER
#define MC_STEPPER

#define GPIO_DIR_Z 4
#define GPIO_PULSE_Z 1
#define GPIO_DIR_Y 5
#define GPIO_PULSE_Y 38
#define GPIO_DIR_X 6
#define GPIO_PULSE_X 15

// TODO: Add this to GPIO_MOTOR_PIN_SEL and uncomment enable
// #define GPIO_STP_ENABLE 7

#define GPIO_MOTOR_PIN_SEL  ( \
    (1ULL<<GPIO_DIR_Z) | (1ULL<<GPIO_PULSE_Z) | \
    (1ULL<<GPIO_DIR_Y) | (1ULL<<GPIO_PULSE_Y) | \
    (1ULL<<GPIO_DIR_X) | (1ULL<<GPIO_PULSE_X)  \
)

// limit switch dimensions
#define LIMIT_X_MIN_DIST 0
#define LIMIT_X_MAX_DIST 500
#define LIMIT_Y_MIN_DIST 0
#define LIMIT_Y_MAX_DIST 282
#define LIMIT_Z_MIN_DIST 327
#define LIMIT_Z_MAX_DIST 690

#define LEN_PAINT_BRUSH 0
#define Z_POLY_COEF_2 -0.009278768
#define Z_POLY_COEF_1 1.705782683
#define Z_POLY_COEF_0 3386.926986769

#define X_DIST_PER_STEP (600.0/2002)
#define Y_DIST_PER_STEP (280.0/910)

extern float end_effector_position[3];

void init_stepper_mc();
uint z_dist_2_steps(uint tip_z);
void step(uint8_t pin);

int move_x(int delta);
int move_y(int delta);
int move_z(uint crnt, uint desired);
void move_stepper(uint8_t motor, float ideal_pos);
void reset_xyz();

#endif // h
