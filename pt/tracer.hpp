#ifndef _TRACER_
#define _TRACER_
#include "scene.hpp"
#include "photon.hpp"
#include "ray.hpp"
using namespace std;
class PMtracer {
public:
    Scene* scene;
    PhotonMap* photonmap;
    int emit_num;
    PMtracer(int emit_num_,Scene* scene_,PhotonMap* pm_):emit_num(emit_num_),scene(scene_),photonmap(pm_){}

    void tracing(const Photon& photon, int dep, unsigned short *X) {
        Intersection res;
        Ray r(photon);
        //photon.power.print();
        //cout << "finding " << endl;
        if (!scene->findNearest_naive(r,res)) {
            //cout << "not find" << endl;
            return;
        }
        //cout << "finded id: " << res.id << endl;

        Object* obj = scene->getObj(res.id);
        V3 color = obj->material.color(res.a,res.b);
        V3 x = r.pos(res.t),n = res.n;


        if (obj->material.refl == DIFF) {
            double p = color.max();
            if (erand48(X) < p) color /= p;
            else return;


            //存储光子
            photonmap->store(photon);

		    double r1 = 2 * PI * erand48(X), r2 = erand48(X), r2s = sqrt(r2);
		    V3 w = n, u = ((fabs(w[0]) > EPS? V3(0,1):V3(1)) & w).norm(), v = w & u;
		    V3 d = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2)).norm();

            //漫反射
            Photon pt;
            pt.pos = x;
            pt.dir = d;
            pt.power = photon.power * color ;
            tracing(pt,dep,X);
        } else if (obj->material.refl == SPEC) {
            V3 d = photon.dir.reflect(n);
            Photon pt;
            pt.pos = x;
            pt.dir = d;
            pt.power = photon.power * color ;
            tracing(pt,dep,X);
        } else if (obj->material.refl == REFR) {
            V3 d = photon.dir.refract(n, res.into? 1:obj->material.ns, res.into? obj->material.ns:1);
            V3 refl_d = photon.dir.reflect(n);
            if (d.len2() < EPS) {
                //全反射
                Photon pt(x,refl_d,photon.power*color);
                tracing(pt,dep,X);
            } else {
			    double a = obj->material.ns - 1;
                double b = obj->material.ns + 1;
                double R0 = a * a / (b * b);
                double c = 1 - (res.into? -r.d.dot(n):-d.dot(n)); 
			    double Re = R0 + (1 - R0) * c * c * c * c * c;
                double Tr = 1 - Re;
                double P = 0.25 + 0.5 * Re;
                double RP = Re / P;
                double TP = Tr / (1 - P); 
                if (erand48(X) < P) {
                    tracing(Photon(x,refl_d,photon.power*color*RP),dep,X);
                } else {
                    tracing(Photon(x,d,photon.power*color*TP),dep,X);
                }
            }
        }

    }

    void run() {
        std::cout << "emit num: " << emit_num << std::endl;
        for (int i = 0;i < emit_num; ++i) {
            //std::cout << "emit index:" << i << std::endl;
            unsigned short X[3] = {i + 1, i * i + 10, i * i * i + 100};
            Photon pt = scene->emitPhoton(emit_num);
            //std::cout << "tracing start" << std::endl;
            tracing(pt,0,X);
        }
        photonmap->build();
        std::cout << "photon tree build done" << std::endl;
        std::cout << "pm photonnum: " << this->photonmap->getStoreNum() << std::endl;
    }
    
};



#endif