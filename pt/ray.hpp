#ifndef _RAY_
#define _RAY_
#include "v3.hpp"
#include "photon.hpp"
struct Ray{
    V3 o,d;//origin,direction
    Ray(V3 o_,V3 d_):o(o_),d(d_.norm()){}
    //photon mapping
    Ray(const Photon& pt):o(pt.pos),d(pt.dir){}
    V3 pos(double t) const {return o + d * t;}
    void print() const {
        std::cout << "-------" << std::endl;
        std::cout << "[Ray]:" << std::endl;
        o.print();
        d.print();
        std::cout << "-------" << std::endl;
    }
};

#endif