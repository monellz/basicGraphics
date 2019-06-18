#ifndef _CUBE_
#define _CUBE_

#include "obj.hpp"
#include "ray.hpp"
#include "utils.hpp"


class Cube: public Object{
public:
    V3 p[2]; //两个对角顶点的位置  p0 < p1

    Cube(int id_,V3 p0,V3 p1, V3 e_, V3 c_, Refl_t refl_, double ns_ = 1.5):
        Object(id_,e_,c_,refl_,ns_){
            p[0] = p0;
            p[1] = p1;
        }
    Cube(int id_,V3 p0,V3 p1, V3 e_, std::string fn_, Refl_t refl_, double ns_ = 1.5):
        Object(id_,e_,fn_,refl_,ns_){
            p[0] = p0;
            p[1] = p1;
    }


    bool intersect(const Ray& r, Intersection& res) override {
        //等价与aabb盒相交
        double tmin[3],tmax[3];
        for (int i = 0;i < 3; ++i) {
            if (fabs(r.d[i]) > EPS) {
                double t1 = (p[0][i] - r.o[i]) / r.d[i];
                double t2 = (p[1][i] - r.o[i]) / r.d[i];
                tmin[i] = min(t1,t2);
                tmax[i] = max(t1,t2);
            } else {
                tmin[i] = 1e15;
                tmax[i] = 1e30;
            }
        }

        double tmin_ = max(tmin[0],tmin[1],tmin[2]);
        double tmax_ = min(tmax[0],tmax[1],tmax[2]);
        if (tmin_ >= tmax_) return false;


        V3 pos = r.pos(tmin_);
        res.n = normal(pos);
        res.t = tmin_;
        res.id = id;
        res.into = (res.n.dot(r.d) < 0)? true:false;
        
        //纹理!
        res.a = -(pos.x - p[0].x) / (p[1].x - p[0].x);
        res.b = -(pos.y - p[0].y) / (p[1].y - p[0].y);
        return true;
    }

    V3 normal(const V3& pos) {
        V3 n(1,1,1);
        if (pos.x > p[0].x + EPS && pos.x < p[1].x - EPS) n.x = 0;
        else {
            if (fabs(pos.x - p[0].x) < EPS) n.x = -1;
            else n.x = 1;
        }
        if (pos.y > p[0].y + EPS && pos.y < p[1].y - EPS) n.y = 0;
        else {
            if (fabs(pos.y - p[0].y) < EPS) n.y = -1;
            else n.y = 1;
        }
        if (pos.z > p[0].z + EPS && pos.z < p[1].z - EPS) n.z = 0;
        else {
            if (fabs(pos.z - p[0].z) < EPS) n.z = -1;
            else n.z = 1;
        }
        
        if (fabs(n.len()) < EPS) std::cout << "error!!! : " << fabs(n.len()) << std::endl;
        return n.norm();
    }

    std::pair<V3, V3> aabb() const override {
        return std::make_pair(p[0],p[1]);
    }
};





#endif