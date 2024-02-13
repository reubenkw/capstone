#include "frame_transform.h"
#include "point.h"
#include "dimensions.h"

/*
reference frames :
    camera (3D with depth) : how objects appear on the captured image origin is the top-left of the image
    arm (2D) : origin is when the contact points to the limit switch
    robot (3D) : origin is the back-left-bottom(ground) of the robot
    global (3D) : the robot reference frame at the start of the program
*/ 

// made easier because the camera gives depth to the camera plane (ie z = depth)
Point3D camera2robot(Point3D camera_point, double arm_x, double arm_y) {
    return Point3D(0,0,0);
}
