#ifndef _V4_
#define _V4_

#include "utils.hpp"

struct M4;

struct V3 {
    union {
        double v[3];
        struct {
            double x,y,z;
        };
    };

    double& operator[](int i) {return v[i];}
    double operator[](int i) const {return v[i];}

    V3(double x_ = 0, double y_ = 0, double z_ = 0):x(x_),y(y_),z(z_) {}

    double operator*(const V3& a) const {return x * a.x + y * a.y + z * a.z;}
    V3 operator+(const V3& a) const {return V3(x + a.x, y + a.y, z + a.z);}
    V3 operator-(const V3& a) const {return V3(x - a.x, y - a.y, z - a.z);}
    V3 operator&(const V3& a) const {return V3(y * a.z - z * a.y, z * a.x - x * a.z, x * a.y - y * a.x);}
    V3 operator/(double p) const {return V3(x / p, y / p, z / p);}


    double len2() const {return x * x + y * y + z * z;}
    double len() const {return sqrt(x * x + y * y + z * z);}
    V3 norm() const {return *this / this->len();}
};


struct V4 {
    union {
        double v[4];
        struct {
            double x,y,z,w;
        };
    };
    
    double& operator[](int i) {return v[i];}
    double operator[](int i) const {return v[i];}

    V4(double x_ = 0, double y_ = 0, double z_ = 0, double w_ = 0):x(x_),y(y_),z(z_),w(w_){}
    V4(const V3& a):x(a.x),y(a.y),z(a.z),w(1){}

    //点乘
    double operator*(const V4& a) const {return x * a.x + y * a.y + z * a.z + w * a.w;}
    V4 operator+(double p) const {return V4(x + p, y + p, z + p, w + p);}
    V4 operator-(double p) const {return V4(x - p, y - p, z - p, w - p);}
    V4 operator*(double p) const {return V4(x * p, y * p, z * p, w * p);}
    V4 operator/(double p) const {return V4(x / p, y / p, z / p, w / p);}
    V4 operator+(const V4& a) const {return V4(x + a.x, y + a.y, z + a.z, w + a.w);}
    V4 operator-(const V4& a) const {return V4(x - a.x, y - a.y, z - a.z, w - a.w);}

    double len2() const {return x * x + y * y + z * z + w * w;}
    double len() const {return sqrt(x * x + y * y + z * z + w * w);}
    V4 norm() const {return *this / this->len(); }

};

struct M4{
    double dst[16];
    /*
        0  4  8  12
        1  5  9  13
        2  6  10 14
        3  7  11 15
    */

#define Q(i,j) dst[(((j) << 2) + (i))]

    M4() {
        memset(dst,0,sizeof(double) * 16);
    }
    M4(const double* src) {
        memcpy(dst,src,sizeof(double) * 16);
    }
    M4(const V4& p) {
        // p^T p 
        // p为平面参数
        for (int i = 0;i < 4; ++i) {
            for (int j = 0;j < 4; ++j) {
                Q(i,j) = p[i] * p[j];
            }
        }
    }
    double& operator[](int i) {return dst[i];}
    double operator[](int i) const {return dst[i];}

    bool isAffine() const {
        return Q(3,0) == 0.0 && Q(3,1) == 0.0&& Q(3,2) == 0.0 && Q(3,3) == 1.0;
    }

    M4 operator*(const M4& a) const {
        M4 res;
        for (int i = 0;i < 4; ++i) {
            for (int j = 0;j < 4; ++j) {
                for (int k = 0;k < 4; ++k) {
                    res[(j << 2) + i] += dst[(k << 2) + i] * a[(j << 2) + k];
                }
            }
        }
        return res;
    }

    M4& operator+=(const M4& a) {
        for (int i = 0;i < 16; ++i) dst[i] += a.dst[i];
        return *this;
    }

    M4 operator+(const M4& a) const {
        M4 res;
        for (int i = 0;i < 16; ++i) res.dst[i] = this->dst[i] + a.dst[i];
        return res;
    }

    M4& operator=(const M4& m) {
        memcpy(dst,m.dst,sizeof(double) * 16);
        return *this;
    }

    M4 reverse() const {
        //assert(isAffine());
        return this->rev4Affine();    
    }

    double det4Affine() const {
        //assert(isAffine());
        return dst[0] * dst[5] * dst[10] + dst[1] * dst[6] * dst[8] + dst[2] * dst[4] * dst[9] - dst[2] * dst[5] * dst[8] - dst[0] * dst[6] * dst[9] - dst[1] * dst[4] * dst[10];
    }

    M4 rev4Affine() const {
        M4 res;
        double det = this->det4Affine();
        det = 1.0 / det;

        res[0] = det * (dst[5] * dst[10] - dst[6] * dst[9]);
        res[1] = -det * (dst[1] * dst[10] - dst[2] * dst[9]);
        res[2] = det * (dst[1] * dst[6] - dst[2] * dst[5]);

        res[4] = -det * (dst[4] * dst[10] - dst[6] * dst[8]);
        res[5] = det * (dst[0] * dst[10] - dst[2] * dst[8]);
        res[6] = -det * (dst[0] * dst[6] - dst[2] * dst[4]);

        res[8] = det * (dst[4] * dst[9] - dst[5] * dst[8]);
        res[9] = -det * (dst[0] * dst[9] - dst[1] * dst[8]);
        res[10] = det * (dst[5] * dst[0] - dst[1] * dst[4]);

        res[12] = -(res[0] * dst[12] + res[4] * dst[13] + res[8] * dst[14]);
        res[13] = -(res[1] * dst[12] + res[5] * dst[13] + res[9] * dst[14]);
        res[14] = -(res[2] * dst[12] + res[6] * dst[13] + res[10] * dst[14]);

        res[3] = res[7] = res[11] = 0.0;
        res[15] = 1.0;

        return res;
    }
    double quadric(const V3& v0, const V3& v1) const {
        //assert(isAffine());
        //return v0^T Q v1
        return Q(0,0) * v0.x * v1.x +
                2 * Q(0,1) * v0.x * v1.y +
                2 * Q(0,2) * v0.x * v1.z +
                2 * Q(0,3) * v0.x +
                Q(1,1) * v0.y * v1.y +
                2 * Q(1,2) * v0.y * v1.z +
                2 * Q(1,3) * v0.y +
                Q(2,2) * v0.z * v1.z +
                2 * Q(2,3) * v0.z +
                Q(3,3);
    }

    double quadric(const V3& v) const {
        return quadric(v,v);

        /*
        //确保Q为affline
        if (!isAffine()) {
            std::cout << "error! quadric for non Affine matrix" << std::endl;
            exit(1);
        }
        // return: v^T Q v
        return Q(0,0) * v.x * v.x +
                2 * Q(0,1) * v.x * v.y +
                2 * Q(0,2) * v.x * v.z +
                2 * Q(0,3) * v.x + 
                Q(1,1) * v.y * v.y +
                2 * Q(1,2) * v.y * v.z +
                2 * Q(1,3) * v.y +
                Q(2,2) * v.z * v.z +
                2 * Q(2,3) * v.z +
                Q(3,3);
        */
    }

    V3 lastColumn() const {
        return V3(dst[12],dst[13],dst[14]);
    }

    void print() const {
        for (int i = 0;i < 4; ++i) {
            printf("%8.5lf  %8.5lf  %8.5lf  %8.5lf\n", Q(i,0), Q(i,1), Q(i,2), Q(i,3));
        }
    }

#undef M4_
};


#endif