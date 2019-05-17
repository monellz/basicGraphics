#ifndef __UTILS_H__
#define __UTILS_H__

#include <bits/stdc++.h>
#include "utils.hpp"


struct V3{
	double x, y, z;
	V3(double x_=0, double y_=0, double z_=0): x(x_), y(y_), z(z_) {}
	V3 operator-() const {return V3(-x, -y, -z);}
	V3 operator+(const V3&a) const {return V3(x+a.x, y+a.y, z+a.z);}
	V3 operator-(const V3&a) const {return V3(x-a.x, y-a.y, z-a.z);}
	V3 operator&(const V3&a) const {return V3(y*a.z-z*a.y, z*a.x-x*a.z, x*a.y-y*a.x);}
	V3 operator*(const V3&a) const {return V3(x * a.x, y * a.y, z * a.z);}
	V3 operator*(double p) const {return V3(x*p, y*p, z*p);}
	V3 operator/(double p) const {return V3(x/p, y/p, z/p);}

	bool operator==(const V3&a) const {return x==a.x && y==a.y && z==a.z;}
	bool operator!=(const V3&a) const {return x!=a.x || y!=a.y || z!=a.z;}
	V3&operator+=(const V3&a) {return *this = *this + a;}
	V3&operator-=(const V3&a) {return *this = *this - a;}
	V3&operator*=(double p) {return *this = *this * p;}
	V3&operator/=(double p) {return *this = *this / p;}
    V3&operator&=(double p) {return *this = *this & p;}
	//double operator|(const V3&a) const {return x*a.x + y*a.y + z*a.z;}
	double dot(const V3&a) const {return x*a.x + y*a.y + z*a.z;}
	double max() const {return x>y&&x>z?x:y>z?y:z;}
	double len() const {return sqrt(x*x + y*y + z*z);}
	double len2() const {return x*x + y*y + z*z;}
	V3 mult(const V3&a) const {return V3(x*a.x, y*a.y, z*a.z);}
	V3 cross(const V3&a) const {return V3(y*a.z-z*a.y, z*a.x-x*a.z, x*a.y-y*a.x);}
	V3 norm() const {return (*this)/len();}
    bool zero() {return fabs(x) < EPS && fabs(y) < EPS && fabs(z) < EPS;}
	V3 clamp() const {return V3(x>1?1:x<0?0:x, y>1?1:y<0?0:y, z>1?1:z<0?0:z);}
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
	void print() const {std::cout << x << " " << y << " " << z << std::endl;}
};

#endif // __UTILS_H__