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

#define GPIO_STEPPER_MOTOR_PIN_SEL  ( \
    (1ULL<<GPIO_DIR_Z) | (1ULL<<GPIO_PULSE_Z) | \
    (1ULL<<GPIO_DIR_Y) | (1ULL<<GPIO_PULSE_Y) | \
    (1ULL<<GPIO_DIR_X) | (1ULL<<GPIO_PULSE_X)  \
)

// limit switch dimensions
#define LIMIT_X_MIN_DIST 0
#define LIMIT_X_MAX_DIST 520
#define LIMIT_Y_MIN_DIST 0
#define LIMIT_Y_MAX_DIST 310
#define LIMIT_Z_MIN_DIST 327
#define LIMIT_Z_MAX_DIST 718    // 721

#define LEN_PAINT_BRUSH 0
#define Z_POLY_COEF_2 -0.00117988947711789
#define Z_POLY_COEF_1 -6.32395140705953
#define Z_POLY_COEF_0 5208.12282792867

#define X_DIST_PER_STEP (LIMIT_X_MAX_DIST/1736.0)
#define Y_DIST_PER_STEP (LIMIT_Y_MAX_DIST/1024.0)

#define X_STEP_DELAY 1000
#define Y_STEP_DELAY 1000
#define Z_STEP_DELAY 500

extern float end_effector_position[3];
extern uint default_step_delays[3];

void init_stepper_mc();
uint z_dist_2_steps(uint tip_z);
void step(uint8_t pin, uint step_delay);

float move_x(float delta, uint step_delay);
float move_y(float delta, uint step_delay);
int move_z(uint crnt, uint desired, uint step_delay);
bool move_stepper(uint8_t motor, float ideal_pos, int step_delay);
void reset_xyz();
void pollinate();
void pollinate_v2();

#endif // h
