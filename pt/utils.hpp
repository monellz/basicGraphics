#ifndef _UTILS_
#define _UTILS_


#include <bits/stdc++.h>
#include <cmath>

#define EPS (1e-6)
#define INF (1e20)
//#define PI (acos(-1))
#define PI (3.14159265358979323846)


enum Refl_t { DIFF, SPEC, REFR };
inline double ran() {
    return double(rand()) / RAND_MAX;
}

inline int toColor(double x) {return int(.5+ 255 *pow(x<0?0:x>1?1:x,1/2.2));}

inline double min(double x, double y) {return x < y? x:y;}
inline double min(double x, double y, double z) {return min(min(x,y),z);}
inline double max(double x, double y) {return x > y? x:y;}
inline double max(double x, double y, double z) {return max(max(x,y),z);}


const double _minpos = -1000;
const double _maxpos = 1000;



#define PM_MAX_PHOTONS 200000
#define PM_EMIT_PHOTONS 100000
#define PM_R 10


#define PPM_ALPHA 0.7  //ppm reduce系数
#define PPM_MAX_PHOTONS 1000000
#define PPM_EMIT_PHOTONS 1000000
#define PPM_INIT_R2 9 //初始化碰撞点半径平方
#define PPM_ROUND 160 //PPM迭代次数


//景深效果  限pt
//#define DEPTH
#define FOCAL_RAD 5  //景深随机圆盘取点半径
#define FOCAL_DIS 75 

//BEZIER
#define MAX_CONTROL 12
#define MAX_NEWTON_ITER 10
#define MAX_RAND_SEED 20


//体积光
//#define GOD_RAY
#define GOD_RAY_SAMP 10 

#endif