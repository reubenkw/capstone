#ifndef DIMENSIONS_H
#define DIMENSIONS_H

#include <math.h>

#define CARTESIAN_X_MIN 0           // [m]
#define CARTESIAN_X_MAX 0.5         // [m]
#define CARTESIAN_Y_MIN 0           // [m]
#define CARTESIAN_Y_MAX 0.275       // [m]
#define CARTESIAN_Z_MIN 0.327       // [m] 
#define CARTESIAN_Z_MAX 0.69        // [m] 
#define FLOOR_2_CAM 0.807           // [m]

#define CAMERA_2_EFFECTOR_X 0.034   // [m]
#define CAMERA_2_EFFECTOR_Y 0.085   // [m]

// (hopefully not painfully) slow to start
#define IDEAL_SPEED_DRIVE  0.;      // [m/s]
#define IDEAL_SPEED_ARM  0.05       // [m/s]
#define WHEEL_RADIUS  0.0762                                // [m]
#define DRIVE_ENC_2_DIST  (2 * M_PI * WHEEL_RADIUS) / 8     // [m/count]
#define XY_ENC_2_DIST  999                                  // [m/count] 
// TODO: something more complicated for z. probably a linear function.

#define ARM_TOL 0.01        // [m]
#define ROBOT_TOL 0.05      // [m]

#endif //h 
