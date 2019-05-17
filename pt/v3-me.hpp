#ifndef _V3_
#define _V3_

#include "utils.hpp"

struct V3 {
    double x,y,z;
    V3(double x_ = 0,double y_ = 0,double z_= 0):x(x_),y(y_),z(z_){}
    V3 operator = (const V3& p) {
        x = p.x;
        y = p.y;
        z = p.z;
        return *this;
    }
    V3 operator -() const {return V3(-x,-y,-z);}
    V3 operator + (const V3& p) const {return V3(x + p.x, y + p.y, z + p.z);}
    V3 operator += (const V3& p) {return *this = *this + p;}
    V3 operator - (const V3& p) const {return V3(x - p.x, y - p.y, z - p.z);}
    V3 operator -= (const V3& p) {return *this = *this - p;}
    //V3 operator * (const V3& p) const {return V3(y * p.z - z * p.y, z * p.x - x * p.z, x * p.y - y - p.x);}
    V3 operator & (const V3& p) const {return V3(y * p.z - z * p.y, z * p.x - x * p.z, x * p.y - y - p.x);}
    //V3 operator *= (const V3& p) {return *this = *this * p;}
    //V3 operator / (const V3& p) const {return V3(x / p.x, y / p.y, z / p.z);}
    //V3 operator /= (const V3& p) {return *this = *this / p;}

    V3 operator + (double d) const {return V3(x + d, y + d, z + d);}
    V3 operator += (double d) {return *this = *this + d;}
    V3 operator - (double d) const {return V3(x - d, y - d, z - d);}
    V3 operator -= (double d) {return *this = *this - d;}
    V3 operator * (double d) const {return V3(x * d, y * d, z * d);}
    V3 operator *= (double d) {return *this = *this * d;}
    V3 operator / (double d) const {return V3(x / d, y / d, z / d);}
    V3 operator /= (double d) {return *this = *this / d;}

    bool operator == (const V3& p) {return fabs(x - p.x) < EPS && fabs(y - p.y) < EPS && fabs(z - p.z) < EPS;}

    V3 mult(const V3& p) const {return V3(x * p.x, y * p.y, z * p.z);}
    double dot(const V3& p) const {return x * p.x + y * p.y + z * p.z;}

    double len2() const {return x * x + y * y + z * z;}
    double len() const {return sqrt(x * x + y * y + z * z);}
    double dis2(const V3& p) const {return (p - *this).len2();}
    double dis(const V3& p) const {return (p - *this).len();}
    double max() const {return x > y? (x > z? x:z):(y > z? y:z);}
    double min() const {return x < y? (x < z? x:z):(y < z? y:z);}


    V3 norm() const {return *this / len();}
    //void norm() {*this = *this / len();}
    bool zero() {return fabs(x) < EPS && fabs(y) < EPS && fabs(z) < EPS;}

    
    /*
    void rand(double hi = 1) {
        do {
            x = 2 * ran() - 1;
            y = 2 * ran() - 1;
            z = 2 * ran() - 1;
        } while ( len2() > hi && len2() < EPS);
        norm();
    }
    */
    V3 vertical() const {
        V3 v = *this & V3(1,0,0); //测试叉乘
        if (v.zero()) v = V3(0,1,0); //与x轴平行 返回y轴即可
        else v = v.norm();
        return v;
    }
    V3 rotate(const V3& axis, double theta) const {
        V3 v;
        double cost = cos( theta );
        double sint = sin( theta );
        v.x += x * ( axis.x * axis.x + ( 1 - axis.x * axis.x ) * cost );
        v.x += y * ( axis.x * axis.y * ( 1 - cost ) - axis.z * sint );
        v.x += z * ( axis.x * axis.z * ( 1 - cost ) + axis.y * sint );
        v.y += x * ( axis.y * axis.x * ( 1 - cost ) + axis.z * sint );
        v.y += y * ( axis.y * axis.y + ( 1 - axis.y * axis.y ) * cost );
        v.y += z * ( axis.y * axis.z * ( 1 - cost ) - axis.x * sint );
        v.z += x * ( axis.z * axis.x * ( 1 - cost ) - axis.y * sint );
        v.z += y * ( axis.z * axis.y * ( 1 - cost ) + axis.x * sint );
        v.z += z * ( axis.z * axis.z + ( 1 - axis.z * axis.z ) * cost );
        return v;
    }
    V3 reflect(const V3& n) const {
        //n法向量
        return *this - n * (2 * this->dot(n));
    }
    /*
    V3 refract(const V3& n, double nir) {
        //n法向量  nir相对折射率
        V3 v = norm();
        double cosi = -n.dot(v);
        double cosr2 = 1 - (nir * nir) * ( 1 - cosi * cosi);
        if (cosr2 > EPS) return v * nir + n * (nir * cosi - sqrt(cosr2));
        //全反射
        return v.reflect(n);
    }
    */
    V3 refract(const V3& n, double ni,double nr) const {
        double cosi = norm().dot(n);
        double nir = ni / nr;
        double cosr2 = 1 - nir * nir * (1 - cosi * cosi);
        if (cosr2 <= 0) return V3();
        double cosr = sqrt(cosr2);
        if (cosi > 0) cosr = -cosr; //出
        return ((*this) * nir - n * (nir * cosi + cosr)).norm();
    }
    V3 diffuse() {
        //随机漫反射
        V3 v = vertical();
        double theta = acos(sqrt(ran())); //TODO
        double pi = 2 * M_PI * 2;
        return rotate(v, theta).rotate(*this, pi);
    }

    V3 clamp() const{
        //划归到[0,1]
        double x_ = x > 1? 1: (x < 0? 0:x);
        double y_ = y > 1? 1: (y < 0? 0:y);
        double z_ = z > 1? 1: (z < 0? 0:z);
        return V3(x_,y_,z_);
    }

    void print() const {
        std::cout << "(" << x << "," << y << "," << z << ")" << std::endl;
    }

    void input(std::stringstream& fin) {
        fin >> x >> y >> z;
    }
};




#endif