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

#endif