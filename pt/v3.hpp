#ifndef __UTILS_H__
#define __UTILS_H__

#include <bits/stdc++.h>
#include "utils.hpp"


//列向量
struct V3{
	//double x, y, z;
	union {
		double v[3];
		struct {
			double x,y,z;
		};
	};
	//匿名结构体/匿名union
	//支持两种访问
	double& operator[](int i) {return v[i];}
	double operator[](int i) const {return v[i];}
	
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
	int maxAxis() const {
		return x > y && x > z? 0:(y > z? 1:2);
	}
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

	V3 vertical() const {
		V3 v = this->cross(V3(1,0,0));
		if (v.len() < EPS) {
			//与x轴平行
			return V3(0,1,0);
		} else return v;
	}

	static V3 random() {
		return V3(2 * ran() - 1, 2 * ran() - 1, 2 * ran() - 1);
	}

	void print() const {std::cout << x << " " << y << " " << z << std::endl;}
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
V3 max(const V3& a, const V3& b) {
	return V3(max(a.x,b.x),max(a.y,b.y),max(a.z,b.z));
}
V3 min(const V3& a, const V3& b) {
	return V3(min(a.x,b.x),min(a.y,b.y),min(a.z,b.z));
}

V3 max(const V3& a, const V3& b, const V3& c) {
	return V3(max(a.x,b.x,c.x),max(a.y,b.y,c.y),max(a.z,b.z,c.z));
}

V3 min(const V3& a, const V3& b, const V3& c) {
	return V3(min(a.x,b.x,c.x),min(a.y,b.y,c.y),min(a.z,b.z,c.z));
}
#endif // __UTILS_H__