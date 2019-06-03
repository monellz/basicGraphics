#ifndef _OBJ_
#define _OBJ_
#include "v3.hpp"
#include "ray.hpp"
#include "material.hpp"

class Object;

struct Intersection{
    //相交结果
    double t = -1;
    V3 n; //法向量
    bool into = false; 
    int id = -1;
    double a = -1,b = -1; //texture;
};


class Object{
public:
    Material material;
    int id;
    Object(int id_,V3 e_, V3 c_, Refl_t m_,double ns_):id(id_),material(e_,c_,m_,ns_){}
    Object(int id_,V3 e_, std::string fn_, Refl_t m_,double ns_):id(id_),material(e_,fn_,m_,ns_){}
    //virtual double intersect(const Ray& r);
    virtual bool intersect(const Ray& r,Intersection& res) = 0;
    virtual std::pair<V3,V3> aabb() const = 0;
};


#endif

