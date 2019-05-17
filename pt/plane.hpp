#ifndef _PLANE_
#define _PLANE_

#include "obj.hpp"

class Plane: public Object {
public:
    // ax + by + cz = d    
    V3 n; //n = (a,b,c)
    double d;
    Plane(int id_, V3 n_, double d_,V3 e_, V3 c_, Refl_t refl_, double ns_ = 1.5):
        Object(id_,e_,c_,refl_,ns_),n(n_),d(d_){}
    Plane(int id_, V3 n_, double d_,V3 e_, std::string fn_, Refl_t refl_, double ns_ = 1.5):
        Object(id_,e_,fn_,refl_,ns_),n(n_),d(d_){}
    double intersect(const Ray&r) override {

        return 0;
    }
    double intersect(const Ray&r, Intersection& res) override {
        double delta = n.dot(r.d);
        if (delta == 0) return 0; //平行光

        double t = (d - n.dot(r.o)) / delta;
        if (t <= 0) return 0;

        //相交
        if (delta < 0) {
            res.into = true;
            res.n = n.norm();
        } else {
            res.into = false;
            res.n = -n.norm();
        }
        res.id = id;
        //纹理
        //TODO


        return t;
    }
};



#endif