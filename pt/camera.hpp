#ifndef _CAMERA_
#define _CAMERA_

#include "v3.hpp"
#include "ray.hpp"

class Camera {
private:
    V3 o,d;
    V3 cx,cy;
    int w,h; //image w * h
    
public:
    Camera(int w_, int h_, V3 o_, V3 d_):w(w_),h(h_),o(o_),d(d_.norm()) {
        cx = V3(w * 0.5 / h);
        cy = V3(cx & d).norm() * 0.5;

    }

    Ray emit(V3& bias, V3& dir) {
        return Ray(o + bias, dir.norm());
    }

    void setRGB(int x,int y, V3 rbg) {

    }

    ~Camera() {
    }
    
};



#endif