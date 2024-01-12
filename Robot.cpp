#include "Robot.h"

#include <cmath>

Robot::Robot(double robotLength, double robotWidth, double wheelRadius, Camera & camera, double robotPosTol, double armPosTol) 
    : robotLength(robotLength), robotWidth(robotWidth), wheelRadius(wheelRadius), camera(camera), robotPosTol(robotPosTol), armPosTol(armPosTol) {
    // TODO: pid terms need to be determined experimentally
    drive[frontLeft] = MotorController(10, 1, 1, 5, 0);
    drive[backLeft] = MotorController(10, 1, 1, 5, 0);
    drive[frontRight] = MotorController(10, 1, 1, 5, 0);
    drive[backRight] = MotorController(10, 1, 1, 5, 0);

    servoArm[x] = MotorController(10, 1, 1, 5, 0);
    servoArm[y] = MotorController(10, 1, 1, 5, 0);
    servoArm[z] = MotorController(10, 1, 1, 5, 0);

    robotPosition = Point(0, 0, 0);
    armPosition = Point(0, 0, 0);
}

Point Robot::getRobotPosition(){
    return robotPosition;
}

Point Robot::getArmPosition(){
    return armPosition;
}

// how are we going to do this? 
// should it just be called at the end of each robot move function? 
void Robot::updateRobotPosition(){}
void Robot::updateArmPosition(){}

// From MTE 544 Control Lecture Slide 9
double calculate_radius(double delta_x, double delta_y){
    double L = sqrt(delta_x*delta_x+delta_y*delta_y);

    return L*L/2*std::abs(delta_y);
}

// From MTE 544 Modeling III IV Lecture Slide 16
// Pass in -w for left wheel
double Robot::calculate_wheel_speed(double v, double w){
    return 1/wheelRadius* ( v + robotWidth * w / 2 + std::pow(robotLength  * w / 2 , 2) / (v + robotWidth * w / 2) );
}

void Robot::driveRobotForward(Point idealPos){
    Point delta = idealPos - robotPosition;

    while(delta.mag() < robotPosTol){
        
        double r = calculate_radius(delta.x, delta.y);

        // TODO: how to figure ideal v?
        double v = 1;
        double w = v / r;
        double leftWheelSpeed = calculate_wheel_speed(v, -1 * w);
        double rightWheelSpeed = calculate_wheel_speed(v, w);
        
        drive[frontLeft].setIdealSpeed(leftWheelSpeed);
        drive[backLeft].setIdealSpeed(leftWheelSpeed);
        drive[frontRight].setIdealSpeed(rightWheelSpeed);
        drive[backRight].setIdealSpeed(rightWheelSpeed);
        
        updateRobotPosition();

        delta = idealPos - robotPosition;
    }

}

void Robot::moveArm(Point idealPos){
    Point delta = idealPos - armPosition;

    while(delta.mag() < armPosTol){
        double idealSpeedx = 1; 
        double idealSpeedy = 1; 
        double idealSpeedz = 1; 
        // TODO: do we need PID controllers ideal speed of servo arm position? 
        servoArm[x].setIdealSpeed(idealSpeedx);
        servoArm[y].setIdealSpeed(idealSpeedy);
        servoArm[z].setIdealSpeed(idealSpeedz);

        updateArmPosition();

        delta = idealPos - armPosition;
    }
}

// TODO: perform pollination pattern
void Robot::pollinate() { }

// TODO: Fancy schmancy image processing
std::vector<Point> Robot::findFlowerCenters(Image const& image){
    std::vector<Point> flowerCenters;
    return flowerCenters;
}

// TODO: Find center of row
double findYCenterOfPlant(Image const& image){
    return 0;
}

