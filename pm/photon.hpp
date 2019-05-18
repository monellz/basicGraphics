#ifndef _PHOTON_
#define _PHOTON_
#include "v3.hpp"
#include "utils.hpp"
using namespace std;
struct Photon {
    V3 pos;
    short plane;
    unsigned char theta, phi;
    V3 power;
};

struct NearestPhotons {
    int max;
    int found;
    int got_heap;
    V3 pos;
    double* dist2;
    const Photon **index;
};

class PhotonMap {
private:
    Photon* photons;
    int storedPhotons;
    int halfStoredPhotons;
    int maxPhotons;
    int prevScale;

    double costheta[256];
    double sintheta[256];
    double cosphi[256];
    double sinphi[256];

    V3 aabbMin,aabbMax;

    void balanceSegment(Photon **pbal, Photon **porg, const int index,const int start, const int end) {
        //计算新的median
        int median = 1;
        while ((4 * median) <= (end - start + 1)) 
            median += median;
        if ((3 * median) <= (end - start + 1)) {
            median += median;
            median += start - 1;
        } else
            median = end - median + 1;

        //找到轴分割
        int axis = (aabbMax - aabbMin).maxAxis();
        medianSplit(porg,start,end,median,axis);

        pbal[index] = porg[median];
        pbal[index]->plane = axis;

        //递归平衡左右节点
        if (median > start) {
            //左
            if (start < median - 1) {
                const double tmp = aabbMax[axis];
                aabbMax[axis] = pbal[index]->pos[axis];
                balanceSegment(pbal,porg,2 * index, start, median - 1);
                aabbMax[axis] = tmp;
            } else {
                pbal[2 * index] = porg[start];
            }
        }
        if (median < end) {
            //右
            if (median + 1 < end) {
                const double tmp = aabbMin[axis];
                aabbMin[axis] = pbal[index]->pos[axis];
                balanceSegment(pbal,porg,2 * index + 1,median + 1,end);
                aabbMin[axis] = tmp;
            } else {
                pbal[2 * index + 1] = porg[end];
            }
        }
        
    }
    void medianSplit(Photon **p, const int start, const int end, const int median,const int axis) {
        //把数组根据median分成两块
        //比较标准为轴
        int left = start;
        int right = end;
        while (right > left) {
            const double v = p[right]->pos[axis];
            int i = left - 1;
            int j = right;
            for (;;) {
                while (p[++i]->pos[axis] < v){}
                while (p[--j]->pos[axis] > v && j > left) {}
                if (i >= j) break;
                Photon* t = p[i];
                p[i] = p[j];
                p[j] = t;
            }

            Photon *t = p[i];
            p[i] = p[right];
            p[right] = t;

            if (i >= median) right = i -1;
            if (i <= median) left = i + 1;
        }
    }

public:
    PhotonMap(const int maxPt) {
        //最大光子数
        storedPhotons = 0;
        prevScale = 1;
        maxPhotons = maxPt;

        photons = new Photon[maxPhotons + 1];
        aabbMin = V3(1e30,1e30,1e30);
        aabbMax = V3(-1e30,-1e30,-1e30);

        //初始化table
        for (int i = 0;i < 256; ++i) {
            double angle = double(i) * (1 / 256.0) * PI;
            costheta[i] = cos(angle);
            sintheta[i] = sin(angle);
            cosphi[i] = cos(2 * angle);
            sinphi[i] = sin(2 * angle);
        }
    }
    ~PhotonMap() {delete []photons;}
    void store(const V3& power, const V3& pos, const V3& dir) {
        //把光子放入数组中(之后会形成kd树)
        if (storedPhotons > maxPhotons) return;
        //储存数超过限制

        storedPhotons++;
        Photon* const node = &photons[storedPhotons];

        node->pos = pos;
        aabbMin = min(aabbMin,node->pos);
        aabbMax = max(aabbMax,node->pos);

        node->power = power;

        int theta = int(acos(dir[2]) * (256.0 / PI));
        if (theta > 255) node->theta = 255;
        else node->theta = (unsigned char)theta;

        int phi = int(atan2(dir[1],dir[0]) * (256.0 / (2 * PI)));
        if (phi > 255) node->phi = 255;
        else if (phi < 0)
            node->phi = (unsigned char)(phi + 256);
        else
            node->phi = (unsigned char)phi;
    }
    void scalePhotonPower(const double scale) {
        //调整power
        // scale = 1 / (# emmited photons)
        for (int i = prevScale; i <= storedPhotons; ++i) {
            photons[i].power *= scale;
        }
        prevScale = storedPhotons;

    }
    void balance() {
        //从数组创建左平衡 kd树
        //在光子图用于渲染之前使用
        if (storedPhotons > 1) {
            Photon **pa1 = new Photon*[storedPhotons + 1];
            Photon **pa2 = new Photon*[storedPhotons + 1];

            for (int i = 0;i <= storedPhotons; ++i) pa2[i] = &photons[i];

            balanceSegment(pa1,pa2,1,1,storedPhotons);
            delete []pa2;

            //重新组织平衡kd树
            int d,j = 1, foo = 1;
            Photon foo_photon = photons[j];
            for (int i = 1; i <= storedPhotons; ++i) {
                d = pa1[j] - photons;
                pa1[j] = nullptr;
                if (d != foo) photons[j] = photons[d];
                else {
                    photons[j] = foo_photon;

                    if (i < storedPhotons) {
                        for (; foo <= storedPhotons; foo++){
                            if (pa1[foo] != nullptr) break;
                        }
                        foo_photon = photons[foo];
                        j = foo;
                    }
                    continue;
                }
                j = d;
            }
            delete []pa1;
        }
        halfStoredPhotons = storedPhotons / 2 - 1;
    }
    void irradianceEstimate(V3& irrad, const V3& pos, const V3& normal, const double maxDist,const int nPhotons) const {
        //估计入射
        //irrad入射光   pos平面位置 normal 法向量 maxDist最大距离  nphotons 使用的光子数
        irrad[0] = irrad[1] = irrad[2] = 0;

        NearestPhotons np;
        np.dist2 = new double[nPhotons + 1];
        np.index = new Photon*[nPhotons + 1];

        np.pos = pos;
        np.max = nPhotons;
        np.found = 0;
        np.got_heap = 0;
        np.dist2[0] = maxDist * maxDist; //平方

        //寻找最近光子
        locatePhotons(&np,1);

        //如果光子少于8个，直接返回
        if (np.found < 8) return;

        V3 pdir;
        //计算所有光子的入射和
        for (int i = 1; i<= np.found; ++i) {
            const Photon* p = np.index[i];
            photonDir(pdir,p);
            if (pdir.dot(normal) < EPS) {
                irrad += p->power;
            }
        }

        //估计密度
        const double tmp = (1.0 / PI) / (np.dist2[0]);

        irrad *= tmp;
    }
    void locatePhotons(NearestPhotons*const np, const int index) const {
        const Photon *p = &photons[index];
        double dist1;

        if (index < halfStoredPhotons) {
            dist1 = np->pos[p->plane] - p->pos[p->plane];

            if (dist1 > EPS) {
                //如果距离为正
                locatePhotons(np,2 * index + 1);
                if (dist1 * dist1 < np->dist2[0])
                    locatePhotons(np,2 * index);
            } else {
                //距离为负
                locatePhotons(np, 2 * index);
                if (dist1 * dist1 < np->dist2[0])
                    locatePhotons(np, 2 * index + 1);
            }
        }
        //计算在当前光子与最近光子pos之间的平方距离
        double dist2 = (p->pos - np->pos).len2();

        if (dist2 < np->dist2[0]) {
            //找到一个光子  插入序列中
            if (np->found < np->max) {
                //堆未满 使用数组
                np->found++;
                np->dist2[np->found] = dist2;
                np->index[np->found] = p;
            } else {
                int j,parent;
                if (np->got_heap == 0) {
                    //需要建堆
                    double dst2;
                    const Photon *phot;
                    int half_found = np->found >> 1; //除以2
                    for (int k = half_found; k>=1; k--) {
                        parent = k;
                        phot = np->index[k];
                        dst2 = np->dist2[k];
                        while (parent <= half_found) {
                            j = parent << 1;
                            if (j < np->found && np->dist2[j] < np->dist2[j + 1])
                                j++;
                            if (dst2 >= np->dist2[j])
                                break;
                            np->dist2[parent] = np->dist2[j];
                            np->index[parent] = np->index[j];
                            parent = j;
                        }
                        np->dist2[parent] = dst2;
                        np->index[parent] = phot;
                    }
                    np->got_heap = 1;
                }

                //插入新的光子到最大堆中
                //删除最大的，插入新的，重排序堆
                parent = 1;
                j = 2;
                while (j <= np->found) {
                    if (j < np->found && np->dist2[j] < np->dist2[j + 1])
                        j++;
                    if (dist2 > np->dist2[j])
                        break;
                    np->dist2[parent] = np->dist2[j];
                    np->index[parent] = np->index[j];
                    parent = j;
                    j += j;
                }
                np->index[parent] = p;
                np->dist2[parent] = dist2;

                np->dist2[0] = np->dist2[1]; //??
            }
        }
    }
    void photonDir(V3& dir, const Photon* p) const {
        //返回光子方向 参数
        dir[0] = sintheta[p->theta] * cosphi[p->phi];
        dir[1] = sintheta[p->theta] * sinphi[p->phi];
        dir[2] = costheta[p->theta]; 
    }
};


#endif