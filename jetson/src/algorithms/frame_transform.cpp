#include "frame_transform.h"
#include "point.h"
#include "constants.h"

/*
reference frames :
    camera (3D with depth) : how objects appear on the captured image origin is the top-left of the image
    robot (3D) : origin is end effector xy at left back limit switch and ground of the robot
    global (3D) : the robot reference frame at the start of the program
*/ 

// made easier because the camera gives depth to the camera plane (ie z = depth)
Point3D camera2robot(Point3D camera_point, double arm_x, double arm_y) {
    double x = arm_x + CAMERA_2_EFFECTOR_X - camera_point.x;
    double y = arm_y + CAMERA_2_EFFECTOR_Y + camera_point.y;
    double z = FLOOR_2_CAM - camera_point.z;

    return Point3D(x,y,z);
}

std::vector<Point3D> camera2robot(std::vector<Point3D> camera_points, double arm_x, double arm_y) {
    std::vector<Point3D> robotPoints;
    for (auto camera_point : camera_points){
        robotPoints.push_back(camera2robot(camera_point, arm_x, arm_y));
    }
    return robotPoints;
}

Point3D robot2global(Point3D robot_point, double robot_x, double robot_y, double robot_angle) {
    double mag = sqrt(robot_point.x * robot_point.x + robot_point.y * robot_point.y);
    double x = robot_x + mag * cos(robot_angle);
    double y = robot_y + mag * sin(robot_angle);

    return Point3D(x,y,robot_point.z);
}
