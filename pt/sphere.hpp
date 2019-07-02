#ifndef _SPHERE_
#define _SPHERE_
#include "v3.hpp"
#include "ray.hpp"
#include "obj.hpp"

using namespace std;

class Sphere: public Object{
public:
    double rad, ns;
    V3 o;
    //Refl_t refl;
    Sphere(int id_,double rad_, V3 o_, V3 e_, V3 c_, Refl_t refl_, double ns_=1.5): 
        Object(id_,e_,c_,refl_,ns_),
        rad(rad_), ns(ns_), o(o_){}
    Sphere(int id_,double rad_, V3 o_, V3 e_, std::string fn_, Refl_t refl_, double ns_ = 1.5):
        Object(id_,e_,fn_,refl_,ns_),
        rad(rad_), ns(ns_), o(o_){}


    //返回0 不相交
    bool intersect(const Ray&r, Intersection& res) override{
        V3 ro = o - r.o;
        double b = r.d.dot(ro);
        double d = b * b - ro.dot(ro) + rad * rad;
        if (d < 0) return false;
        else d = sqrt(d);
        double t = (b - d > EPS)? b-d : ((b + d) > EPS? b + d : 0);
        if (t == 0) return false;
        res.t = t;
        V3 x = r.pos(t);
        V3 n = (x - o).norm(); //法向量
        if (n.dot(r.d) < 0) {
            res.into = true;
            res.n = n;
        } else {
            res.into = false;
            res.n = -n;
        }
        res.id = id;

        
        //纹理
        V3 pts = (x - o) / rad;
        if (pts.x == 0 && pts.y == 0) {
            res.a = res.b = 0;
        } else {
            res.a = atan2(pts.y,pts.x) / (2 * PI);
            res.b = asin(pts.z) / (2 * PI);
        }

        return true;
    }

    std::pair<V3, V3> aabb() const override {
        return std::make_pair(o - rad - EPS, o + rad + EPS); 
    }

    Ray light() {
        //向y轴负发射光线/光子
        double x = 2 * ran() - 1;
        double y = -ran();
        double z = 2 * ran() - 1;
        Ray r(V3(),V3(x,y,z));
        r.o = o + r.d * (rad + 2 * EPS);
        return r;
    }
};

#endif