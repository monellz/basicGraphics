#ifndef _CAMERA_
#define _CAMERA_

#include "v3.hpp"
#include "ray.hpp"

class Camera {
private:
    V3 o,d;
    V3 cx,cy;
    int w,h; //image w * h
    
    //output
    V3* img = nullptr;
public:
    Camera(int w_, int h_, V3 o_, V3 d_):w(w_),h(h_),o(o_),d(d_.norm()) {
        cx = V3(w * 0.5 / h);
        cy = V3(cx & d).norm() * 0.5;

        img = new V3[w * h];
    }

    Ray emit(V3& bias, V3& dir) {
        return Ray(o + bias, dir.norm());
    }

    void setRGB(int x,int y, V3 rbg) {
        img[x * w + y] = rbg;
    }

    void dump(std::string fn) {
        FILE* fp = fopen(fn.c_str(),"w");
        fprintf(fp,"P6\n%d %d\n%d\n",w,h,255);
        for(int y=h-1;y>=0;--y)
            for(int x=w-1;x>=0;--x)
                fprintf(fp,"%c%c%c",toColor(img[y*w+x][0]),toColor(img[y*w+x][1]),toColor(img[y*w+x][2]));
    }
    ~Camera() {
        if (img != nullptr) delete img;
    }
    
};



#endif