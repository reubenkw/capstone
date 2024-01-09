#ifndef IMAGING_H
#define IMAGING_H

const int IMAGE_WIDTH = 1280;
const int IMAGE_HEIGHT = 720;

struct Pixel {
    double r;
    double g;
    double b;
};

class Image {
    Pixel image [IMAGE_WIDTH][IMAGE_HEIGHT];
    public:
        Pixel readPixelVal(int x, int y);
        void writePixelVal(int x, int y, Pixel val);
};

class Camera {
    public: 
        Camera();
        Image getCameraImage(); 
        double getDepthVal(int x, int y);
};

#endif // h