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
	double intersect(const Ray&r) override{
		V3 ro = o-r.o;
		double b = r.d.dot(ro);
		double d = b*b-ro.dot(ro)+rad*rad;
		if (d<0) return 0;
		else d=sqrt(d);
		return b-d>EPS ? b-d : b+d>EPS? b+d : 0;
	}

    //返回0 不相交
	bool intersect(const Ray&r, Intersection& res) override{
		V3 ro = o-r.o;
		double b = r.d.dot(ro);
		double d = b*b-ro.dot(ro)+rad*rad;
		if (d<0) return false;
		else d=sqrt(d);
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
		double p = acos(-n.dot(V3(0,0,1))); //z轴
		double q = acos(min(max(n.dot(V3(1,0,0)) / sin(p), -1.0), 1.0));
		double u = p / PI;
		double v = q / 2 / PI;
		if (n.dot(V3(0,1,0)) < 0) v = 1 - v;
		res.a = u;
		res.b = v;

        return true;
		//return b-d>EPS ? b-d : b+d>EPS? b+d : 0;
	}

	std::pair<V3, V3> aabb() const override {
		return std::make_pair(o - rad - EPS, o + rad + EPS); //EPS必须加????
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