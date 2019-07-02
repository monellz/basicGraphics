#ifndef _PPM_
#define _PPM_

#include "render.hpp"
#include "hit.hpp"
#include "v3.hpp"
using namespace std;

class PPM: public Render {
private:
    HitPointMap* hm;
    PhotonMap* pm;


    //raytrace
    void raytrace(const Ray& r,int i,int j,V3 factor, int dep, unsigned short *X) {

        Intersection res;
        //cout << "solve" << endl;
        if(!scene->findNearest_naive(r,res))return;
        //cout << "intersect" << endl;
        Object* obj = scene->getObj(res.id);
        //cout << "id: " << res.id << endl;

        V3 x = r.pos(res.t), n = res.n,color = obj->material.color(res.a,res.b);

        double p = color.max();
        if(++dep> 5) {
            if (dep > 25) return;
            if(erand48(X)<p) color /= p;
            else return;
        }
        if(obj->material.refl==DIFF){
            //cout << "into diff" << endl;
            
            HitPoint hp;
            hp.pos = x;
            //hp.x = i;
            //hp.y = j;
            hp.index = i * w + j;
            hp.color = obj->material.e + color * factor;
            hm->store(hp);
            return;
        }
        else{
            Ray reflray = Ray(x,r.d.reflect(n));
            if (obj->material.refl == SPEC){
                raytrace(reflray,i,j, color * factor, dep, X); 
                return;
            }
            else{
                //V3 d = r.d.refract(n, into?1:obj.ns, into?obj.ns:1); //...
                V3 d = r.d.refract(n, res.into?1:obj->material.ns, res.into?obj->material.ns:1); //...
                if (d.len2()<EPS) { // Total internal reflection 
                    raytrace(reflray,i,j, color * factor, dep, X); 
                    return;
                }
                double a = obj->material.ns - 1;
                double b = obj->material.ns + 1;
                double R0 = a * a / (b * b);
                double c = 1 - (res.into? -r.d.dot(n):-d.dot(n)); 
                double Re = R0 + (1 - R0) * c * c * c * c * c;
                double Tr = 1 - Re;
                double P = 0.25 + 0.5 * Re;
                double RP = Re / P;
                double TP = Tr / (1-P); 
                if (dep > 2) {
                    if (erand48(X) < P) raytrace(reflray,i,j,color * factor * RP,dep,X);
                    else raytrace(Ray(x,d),i,j,color * factor * TP, dep, X);
                } else {
                    raytrace(reflray,i,j,color * factor * Re, dep, X);
                    raytrace(Ray(x,d),i,j,color * factor * Tr, dep, X);
                }
            }
        }
    }
    void photontrace(const Photon& photon, V3 factor, int dep, unsigned short *X) {
        Intersection res;
        Ray r(photon);
        //photon.power.print();
        //cout << "finding " << endl;
        if (!scene->findNearest_naive(r,res)) {
            //cout << "not find" << endl;
            return;
        }
        if (res.id < 0) return;
        if (dep++ > 20) return;

        Object* obj = scene->getObj(res.id);
        V3 color = obj->material.color(res.a,res.b);
        V3 x = r.pos(res.t),n = res.n;


        if (obj->material.refl == DIFF) {
            double p = color.max(); 
            if (erand48(X) < p) color /= p;
            else return;


            //更新附近hitpoint
            Nearest<HitPoint> nh(x,sqrt(PPM_INIT_R2));
            hm->searchHitPoints(nh);
            for (int i = 0;i < nh.items.size(); ++i) {
                HitPoint* hp = nh.items[i];
                if ((hp->pos - x).len2() > hp->r2) continue;
                hm->reduce(hp,photon.power * hp->color,1);
                //cout << "flux: " << endl;
                //hp->flux.print();
            }

            double r1 = 2 * PI * erand48(X);
            double r2 = erand48(X);
            double r2s = sqrt(r2);
            //找垂直
            V3 w = n, u=((fabs(w[0]) > EPS ? V3(0,1):V3(1))&w).norm(), v = w & u;
            V3 d = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2)).norm();

            //漫反射
            Photon pt;
            pt.pos = x;
            pt.dir = d;
            pt.power = photon.power * color ;
            photontrace(pt,color * factor,dep,X);
        } else if (obj->material.refl == SPEC) {
            V3 d = photon.dir.reflect(n);
            Photon pt;
            pt.pos = x;
            pt.dir = d;
            pt.power = photon.power * color ;
            photontrace(pt,color * factor,dep,X);
        } else if (obj->material.refl == REFR) {
            V3 d = photon.dir.refract(n, res.into? 1:obj->material.ns, res.into? obj->material.ns:1);
            V3 refl_d = photon.dir.reflect(n);
            if (d.len2() < EPS) {
                //全反射
                Photon pt(x,refl_d,photon.power*color);
                photontrace(pt,color * factor, dep,X);
            } else {
                double a = obj->material.ns - 1;
                double b = obj->material.ns + 1;
                double R0 = a * a / (b * b);
                double c = 1-(res.into? -r.d.dot(n):-d.dot(n)); 
                double Re = R0 + (1 - R0) * c * c * c * c * c;
                double Tr = 1 - Re;
                double P = 0.25 + 0.5 * Re;
                double RP = Re / P;
                double TP = Tr /(1 - P); 
                if (erand48(X) < P) {
                    photontrace(Photon(x,refl_d,photon.power*color*RP),color * factor,dep,X);
                } else {
                    photontrace(Photon(x,d,photon.power*color*TP),color * factor, dep,X);
                }
            }
        }

    }



public:
    int samp;
    PPM(int w_,int h_,int samp_,Scene* scene_):Render(w_,h_,scene_),samp(samp_){
        hm = new HitPointMap();
        pm = new PhotonMap(PPM_MAX_PHOTONS);
    }
    ~PPM() {
        delete hm;
        delete pm;
    }

    void generateIMGInRound(int round, std::string fn) {
        FILE *f = fopen((fn + "+" + std::to_string(round)).c_str(),"w");
        fprintf(f, "P6\n%d %d\n%d\n", w, h, 255);
        V3* image = new V3[w * h];

        for (int i = 0; i < hm->getStoreNum(); ++i) {
            HitPoint* hp = hm->getHitPoint(i);
            image[hp->index] += hp->flux / (PI * hp->r2 * round * PI);
        }
        for (int i = h - 1;i >= 0; --i) {
            for (int j = w - 1; j >= 0; --j) {
                image[i * w + j] = image[i * w + j].clamp();
                fprintf(f, "%c%c%c", toColor(image[i * w + j][0]), toColor(image[i * w + j][1]), toColor(image[i * w + j][2]));
            }
        } 

        delete [] image;
    }

    void rendering() override {
        cout << "into rendering" << endl;
        Ray cam(V3(70,32,280), V3(-0.15,0.05,-1).norm()); 

        double fr = 0.5; 

        V3 cx = V3(w * fr / h), cy = (cx & cam.d).norm() * fr, r;

        cout << "samp: " << samp * 4 << endl;
        for(int y = 0;y < h;++y){
            fprintf(stderr,"\ruse %d samp  %5.2f%%  hitpoint %d",samp * 4, 100.0 * y / h,hm->getStoreNum());
            for(int x = 0;x < w; ++x){
                //cout << "x: " 0<< x << endl;
                /*
                unsigned short X[3]={y+1,y*x+2,y*x*y+3*4};
                V3 d = cx * (x * 1. / w - 0.5) + cy * (y * 1. / h - 0.5) + cam.d;
                raytrace(Ray(cam.o+d*120,d.norm()),y,x,V3(1,1,1),0,X); //basic
                */
                for(int sy = 0;sy < 2;++sy)
                    for(int sx = 0;sx < 2;++sx)
                    {
                        unsigned short X[3]={y,y * x,sx * sy};
                        r[0]=r[1]=r[2]=0;
                        for(int s = 0;s < samp;++s){
                            double r1 = 2 * erand48(X), dx = r1<1 ? sqrt(r1): 2 - sqrt(2 - r1);
                            double r2 = 2 * erand48(X), dy = r2<1 ? sqrt(r2): 2 - sqrt(2 - r2);
                            V3 d = cx * ((sx + dx / 2 + x)/w - 0.5) + cy * ((sy + dy / 2 + y) / h - 0.5) + cam.d; 
                            raytrace(Ray(cam.o + d * 120,d.norm()),y,x,V3(1,1,1) / (samp * 4.0),0,X); 
                        }
                    }
                
            }
        }
        hm->build();
        cout << "raytrace done" << endl;

        for (int round = 0;round < PPM_ROUND; ++round) {

            for (int i = 0;i < PPM_EMIT_PHOTONS; ++i) {
                fprintf(stderr,"\rround %d  emited %5.2f%%",round,100.0* i / PPM_EMIT_PHOTONS);
                unsigned short X[3] = {i + 1, i + 10, i + 100};
                Photon pt = scene->emitPhoton(PPM_EMIT_PHOTONS);
                //pt.power = pt.power * 2500 * PI * 4;
                photontrace(pt,V3(1,1,1),0,X);
            } 
            
            //if (round % 5 == 0) generateIMGInRound(round,"ppm");
            generateIMGInRound(round,"ppm");
        }
        //最终估计
        V3* tmp = new V3[w * h];
        for (int i = 0; i < hm->getStoreNum(); ++i) {
            HitPoint* hp = hm->getHitPoint(i);
            //int x = hp->x;
            //int y = hp->y;
            //cout << "x,y: "<< x << ", " << y << endl;

            //img[x * w + y] = img[x * w + y] + hp->flux / (PI * hp->r2 * PPM_EMIT_PHOTONS);
            img[hp->index] = img[hp->index] + hp->flux / (PI * hp->r2 * PPM_ROUND * PI);
            //多除以一个PI控制光亮度

        }
        
        for (int i = 0;i < h; ++i) {
            for (int j = 0;j < w; ++j) {
                img[i * w + j] = img[i * w + j].clamp();
            }
        }
        
    }
};


#endif