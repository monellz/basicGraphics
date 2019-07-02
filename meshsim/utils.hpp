#ifndef _UTILS_
#define _UTILS_

#include <cmath>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <vector>


//DEBUG
//#define NDEBUG
#include <assert.h>

#define EPS 1e-6
#define INF 1e30

#define LOOP_INFINIT 100000

#define DEFAULT_THRESHOLD 0

template<class T>
inline T min(const T& a, const T& b) {
    return a < b? a:b;
}

template<class T>
inline T max(const T& a, const T& b) {
    return a > b? a:b;
}

#endif