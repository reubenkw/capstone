#ifndef DIMENSIONS_H
#define DIMENSIONS_H

const float CARTESIAN_X_MIN = 0;        // [m]
const float CARTESIAN_X_MAX = 0.620;    // [m]
const float CARTESIAN_Y_MIN = 0;        // [m]
const float CARTESIAN_Y_MAX = 0.330;    // [m]
const float CARTESIAN_Z_MIN = 0;        // [m] won't actually be zero
const float CARTESIAN_Z_MAX = 0.82;     // [m] very approx
const float FLOOR_2_CAM = 0.890;        // [m]

const float CAMERA_2_EFFECTOR_X = 0.032;    // [m]
const float CAMERA_2_EFFECTOR_Y = 0.058;    // [m]

const float DRIVE_ENC_2_DIST = 999; // [m/count]
const float XY_ENC_2_DIST = 999;    // [m/count]

const float WHEEL_RADIUS = 0.0762; // [m]

const float ARM_TOL = 0.01; // [m]
const float ROBOT_TOL = 0.05; // [m]

#endif //h 
