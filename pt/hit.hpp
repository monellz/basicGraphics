#ifndef _HIT_
#define _HIT_
#include "kdtree-tem.hpp"


struct HitPoint {
    V3 pos;

    //int x,y; //该碰撞点对应的图像坐标
    int index; //对应图像坐标 x * w + y

    //brdf
    //pixel position

    V3 color; //颜色?


    double r2; //当前光子半径平方
    unsigned int cnt; //累计光子数目
    V3 flux; //累计反射光通量

    HitPoint():index(0),r2(PPM_INIT_R2){}
};


class HitPointMap {
private:
    std::vector<HitPoint> hitpoints;   
    Tree<HitPoint> tree;
public:
    HitPointMap() {}

    void store(const HitPoint& hitpoint) {
        hitpoints.push_back(hitpoint);
    }

    int getStoreNum() const {
        return hitpoints.size();
    }

    void build() {
        tree.build(hitpoints.data(),hitpoints.size());
    }

    void searchHitPoints(Nearest<HitPoint>& np) {
        tree.search(np);
    }

    HitPoint* getHitPoint(int index) {
        return &hitpoints[index];
    }

    void reduce(HitPoint* hp, const V3& new_flux, int num) {
        //更新这个一个hitpoint
        double f2 = (hp->cnt + PPM_ALPHA * num) / (hp->cnt + num);
        hp->r2 *= f2;
        hp->cnt += num;
        hp->flux = (hp->flux + new_flux) * f2;
    }
};






#endif