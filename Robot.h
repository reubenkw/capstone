#ifndef ROBOT_H
#define ROBOT_H

#include "MotorController.h"
#include "Imaging.h"

#include <math.h> 

enum DriveMotors {frontLeft, backLeft, frontRight, backRight};
enum ServoMotors {x, y, z};

struct Point {
    double x;
    double y;
    double z;
    
    Point(double x = 0, double y = 0, double z = 0): x(x), y(y), z(z) {}
    bool operator==(Point const& rhs)  { return x ==rhs.x && y == rhs.y && z == rhs.z; }
    bool operator!=(Point const& rhs)  { return !(*this == rhs); }
    Point operator+(Point const& rhs)  { return Point(x + rhs.x, y + rhs.y, z + rhs.z); }
    Point operator-(Point const& rhs)  { return Point(x - rhs.x, y - rhs.y, z - rhs.z); }
    double mag() {return sqrt(x*x + y*y + z*z); }
};

class Robot {
    double robotLength;
    double robotWidth;
    double wheelRadius;

    double robotPosTol;
    double armPosTol;

    MotorController drive[4];
    MotorController servoArm[3];

    // there's only one camera, everything should affect the same camera
    Camera & camera; 

    Point robotPosition;
    Point armPosition;
    public: 
        

        Robot(double robotLength, double robotWidth, double wheelRadius, Camera & camera, double robotPosTol, double armPosTol);
        Point getRobotPosition();
        void updateRobotPosition();
        Point getArmPosition();
        void updateArmPosition();

        double calculate_wheel_speed(double v, double w);
        void driveRobotForward(Point idealPos);
        void moveArm(Point idealPos);
        void pollinate();
        std::vector<Point> findFlowerCenters(Image const& image);
        double findYCenterOfPlant(Image const& image);
        // TODO: add logging somewhere
};

#endif // h