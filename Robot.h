#ifndef ROBOT_H
#define ROBOT_H

const int IMAGE_WIDTH = 1280;
const int IMAGE_HEIGHT = 720;

class RGBImage {
    int rgbImage [3][IMAGE_WIDTH][IMAGE_HEIGHT];
};


class Camera {
    public: 
        Camera();
        RGBImage getCameraImage(); 
};

class Robot {
    double robotLength;
    double robotWidth;
    double wheelRadius;


};

#endif // h