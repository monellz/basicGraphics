#ifndef __UTILS_H__
#define __UTILS_H__

#include <bits/stdc++.h>
#include "utils.hpp"


//列向量
struct V3{
	union {
		double v[3];
		struct {
			double x,y,z;
		}sub;
	}crd;
	//使得支持两种访问


	//double x, y, z;
	V3(double x_=0, double y_=0, double z_=0) {
		crd.sub.x = x_;
		crd.sub.y = y_;
		crd.sub.z = z_;
	}
	double& operator[](int i) {
		return crd.v[i];
	}
	double operator[](int i) const {
		return crd.v[i];
	}

	V3 operator-() const {return V3(-crd.sub.x, -crd.sub.y, -crd.sub.z);}
	V3 operator+(const V3&a) const {return V3(crd.sub.x+a.crd.sub.x, crd.sub.y+a.crd.sub.y, crd.sub.z+a.crd.sub.z);}
	V3 operator-(const V3&a) const {return V3(crd.sub.x-a.crd.sub.x, crd.sub.y-a.crd.sub.y, crd.sub.z-a.crd.sub.z);}
	V3 operator&(const V3&a) const {return V3(crd.sub.y*a.crd.sub.z-crd.sub.z*a.crd.sub.y, crd.sub.z*a.crd.sub.x-crd.sub.x*a.crd.sub.z, crd.sub.x*a.crd.sub.y-crd.sub.y*a.crd.sub.x);}
	V3 operator*(const V3&a) const {return V3(crd.sub.x * a.crd.sub.x, crd.sub.y * a.crd.sub.y, crd.sub.z * a.crd.sub.z);}
	V3 operator*(double p) const {return V3(crd.sub.x*p, crd.sub.y*p, crd.sub.z*p);}
	V3 operator/(double p) const {return V3(crd.sub.x/p, crd.sub.y/p, crd.sub.z/p);}

	bool operator==(const V3&a) const {return crd.sub.x==a.crd.sub.x && crd.sub.y==a.crd.sub.y && crd.sub.z==a.crd.sub.z;}
	bool operator!=(const V3&a) const {return crd.sub.x!=a.crd.sub.x || crd.sub.y!=a.crd.sub.y || crd.sub.z!=a.crd.sub.z;}
	V3&operator+=(const V3&a) {return *this = *this + a;}
	V3&operator-=(const V3&a) {return *this = *this - a;}
	V3&operator*=(double p) {return *this = *this * p;}
	V3&operator/=(double p) {return *this = *this / p;}
    V3&operator&=(double p) {return *this = *this & p;}
	//double operator|(const V3&a) const {return crd.sub.x*a.crd.sub.x + crd.sub.y*a.crd.sub.y + crd.sub.z*a.crd.sub.z;}
	double dot(const V3&a) const {return crd.sub.x*a.crd.sub.x + crd.sub.y*a.crd.sub.y + crd.sub.z*a.crd.sub.z;}
	double max() const {return crd.sub.x>crd.sub.y&&crd.sub.x>crd.sub.z?crd.sub.x:crd.sub.y>crd.sub.z?crd.sub.y:crd.sub.z;}
	double len() const {return sqrt(crd.sub.x*crd.sub.x + crd.sub.y*crd.sub.y + crd.sub.z*crd.sub.z);}
	double len2() const {return crd.sub.x*crd.sub.x + crd.sub.y*crd.sub.y + crd.sub.z*crd.sub.z;}
	V3 mult(const V3&a) const {return V3(crd.sub.x*a.crd.sub.x, crd.sub.y*a.crd.sub.y, crd.sub.z*a.crd.sub.z);}
	V3 cross(const V3&a) const {return V3(crd.sub.y*a.crd.sub.z-crd.sub.z*a.crd.sub.y, crd.sub.z*a.crd.sub.x-crd.sub.x*a.crd.sub.z, crd.sub.x*a.crd.sub.y-crd.sub.y*a.crd.sub.x);}
	V3 norm() const {return (*this)/len();}
    bool zero() {return fabs(crd.sub.x) < EPS && fabs(crd.sub.y) < EPS && fabs(crd.sub.z) < EPS;}
	V3 clamp() const {return V3(crd.sub.x>1?1:crd.sub.x<0?0:crd.sub.x, crd.sub.y>1?1:crd.sub.y<0?0:crd.sub.y, crd.sub.z>1?1:crd.sub.z<0?0:crd.sub.z);}
	V3 reflect(const V3&n) const {return (*this)-n*2.*n.dot(*this);}
	V3 refract(const V3&n, double ni, double nr) const { // smallPT1.ppt Page#72
		double cosi = this->norm().dot(n);
		double nir = ni / nr;
		double cosr2 = 1. - nir*nir*(1-cosi*cosi);
		if (cosr2 <= 0)
			return V3();
		double cosr = sqrt(cosr2);
		if (cosi > 0) // out
			cosr = -cosr;
		return ((*this)*nir - n*(nir*cosi+cosr)).norm();
	}
	void print() const {std::cout << crd.sub.x << " " << crd.sub.y << " " << crd.sub.z << std::endl;}
};


//计算行列式 det(b0,b1,b2) bi为列向量
double det(const V3& b0,const V3& b1, const V3& b2) {
	/*
		b00, b10, b20
		b01, b11, b21
		b02, b12, b22
	*/
	//对第三列展开
	double ld = b0[1] * b1[2] - b0[2] * b1[1];
	double lm = b0[0] * b1[2] - b1[0] * b0[2];
	double lu = b0[0] * b1[1] - b1[0] * b0[1]; 

	double res = b2[0] * ld - b2[1] * lm + b2[2] * lu;
	return res;
}

#endif