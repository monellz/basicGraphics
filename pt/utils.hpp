#ifndef _UTILS_
#define _UTILS_


#include <bits/stdc++.h>
#include <cmath>

#define EPS (1e-6)
#define PI (acos(-1))


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



#define PM_MAX_PHOTONS 100000
#define PM_EMIT_PHOTONS 100000


#define PPM_ALPHA 0.7  //ppm reduce系数
#define PPM_MAX_PHOTONS 100000
#define PPM_EMIT_PHOTONS 100000
#define PPM_INIT_R2 25 //初始化碰撞点半径平方
#define PPM_ROUND 1 //PPM迭代次数



#endif