#ifndef _TRAINGLE_
#define _TRAINGLE_
#include "obj.hpp"
#include "ray.hpp"
#include "v3.hpp"

class Triangle: public Object {
public:
    V3 pts[3];
    V3 n; //所指方向为三角片外向 
    Triangle(int id_, V3 p1, V3 p2, V3 p3, V3 n_, V3 e_, V3 c_, Refl_t refl_, double ns_ = 1.5):
        Object(id_,e_,c_,refl_,ns_),n(n_.norm()) {
            pts[0] = p1;
            pts[1] = p2;
            pts[2] = p3;
        }

    bool intersect(const Ray& r, Intersection& res) override {
        if (n.dot(r.d) == 0) return 0;
        //cramer法则求解
        V3 e1 = pts[0] - pts[1];
        V3 e2 = pts[0] - pts[2];
        V3 s = pts[0] - r.o;
        double d12 = det(r.d,e1,e2);
        if (d12 == 0) return false; //无解
        
        double s12 = det(s,e1,e2);
        double ds2 = det(r.d,s,e2);
        double d1s = det(r.d,e1,s);

        double t = s12 / d12;
        double beta = ds2 / d12;
        double gamma = d1s / d12;

        //检查是否合理
        if (t > 0 && beta >= 0 && beta <= 1 && gamma >= 0 && gamma <= 1 && beta + gamma <= 1) {
            //相交

            res.t = t;
            res.id = id;
            res.into = n.dot(r.d) < 0;
            res.n = res.into? n.norm():-n.norm();

            //纹理?

            return true;
        } else return false;
    }
    
    std::pair<V3, V3> aabb() const override{
        return std::make_pair(min(pts[0],pts[1],pts[2]),max(pts[0],pts[1],pts[2]));
    }
};






#endif