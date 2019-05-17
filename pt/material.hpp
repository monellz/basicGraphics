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
    V3 color2(double a,double b) {
        if (buf == nullptr) return c;
	    double U = ( a - floor( a ) ) * h;
    	double V = ( b - floor( b ) ) * w;
    	int U1 = ( int ) floor( U + EPS ) , U2 = U1 + 1;
    	int V1 = ( int ) floor( V + EPS ) , V2 = V1 + 1;
    	double rat_U = U2 - U;
    	double rat_V = V2 - V;
    	if ( U1 < 0 ) U1 = h - 1; if ( U2 == h ) U2 = 0;
	    if ( V1 < 0 ) V1 = w - 1; if ( V2 == w ) V2 = 0;
	    V3 ret;
	    ret = ret + color(a,b) * rat_U * rat_V;
	    ret = ret + color(a,b) * rat_U * ( 1 - rat_V );
	    ret = ret + color(a,b) * ( 1 - rat_U ) * rat_V;
	    ret = ret + color(a,b) * ( 1 - rat_U ) * ( 1 - rat_V );
	    return ret;
    }
};


#endif
