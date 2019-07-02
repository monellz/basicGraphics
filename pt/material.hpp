#ifndef _MATERIAL_
#define _MATERIAL_
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

struct Material {
    Refl_t refl;
    V3 c;
    V3 e;
    double ns;
    std::string fn;
    unsigned char *buf = nullptr;
    int w,h,ch;
    Material(V3 e_,V3 c_,Refl_t refl_,double ns_):refl(refl_),c(c_),buf(nullptr),e(e_),ns(ns_){}
    Material(V3 e_,std::string fn_, Refl_t refl_,double ns_):refl(refl_),fn(fn_),e(e_),ns(ns_){
        if (fn != "") buf = stbi_load(fn.c_str(),&w,&h,&ch,0);
        else buf = nullptr;
    }
    V3 color(double a,double b) {
        if (buf == nullptr) return c;

        int w_ = (int(a * w) % w + w) % w, h_ = (int(b * h) % h + h) % h;
        int idx = h_ * w * ch  + w_ * ch;
        int x = buf[idx + 0], y = buf[idx + 1], z = buf[idx + 2];
        return V3(x,y,z) / 255;
    }
};


#endif
