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
    bool intersect(const Ray&r, Intersection& res) override {
        double delta = n.dot(r.d);
        if (delta == 0) return false; //平行光

        double t = (d - n.dot(r.o)) / delta;
        if (t <= 0) return false;

        //相交
        res.t = t;
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

        /*
        V3 dx = n.vertical();
        V3 dy = n & dx;
        dx = dx.norm() * 2;
        dy = dy.norm() * 2;
        V3 x = r.pos(t);
        */
        V3 x = r.pos(t);
        V3 dx = V3(1,0,0);
        V3 dy = V3(0,1,0);
        res.a = x.dot(dx) / 8;
        res.b = x.dot(dy) / 8;

        return true;
    }
    std::pair<V3, V3> aabb() const override {
        return std::make_pair(V3(), V3());
    }
};



#endif