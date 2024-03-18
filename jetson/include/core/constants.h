#ifndef DIMENSIONS_H
#define DIMENSIONS_H

#include <math.h>

const float CARTESIAN_X_MIN = 0;        // [m]
const float CARTESIAN_X_MAX = 0.6;      // [m]
const float CARTESIAN_Y_MIN = 0;        // [m]
const float CARTESIAN_Y_MAX = 0.275;    // [m]
const float CARTESIAN_Z_MIN = 0.327;    // [m] 
const float CARTESIAN_Z_MAX = 0.69;     // [m] 
const float FLOOR_2_CAM = 0.807;        // [m]

const float CAMERA_2_EFFECTOR_X = 0.034;    // [m]
const float CAMERA_2_EFFECTOR_Y = 0.085;     // [m]

// (hopefully not painfully) slow to start
const float IDEAL_SPEED_DRIVE = 0.1;    // [m/s]
const float IDEAL_SPEED_ARM = 0.05;     // [m/s]

const float WHEEL_RADIUS = 0.0762;                              // [m]
const float DRIVE_ENC_2_DIST = (2 * M_PI * WHEEL_RADIUS) / 8;   // [m/count]
const float XY_ENC_2_DIST = 999;                                // [m/count]
// TODO: something more complicated for z. probably a linear function.

const float ARM_TOL = 0.01;     // [m]
const float ROBOT_TOL = 0.05;   // [m]

#endif //h 
