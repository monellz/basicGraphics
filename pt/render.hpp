#ifndef _RENDER_
#define _RENDER_

#include "v3.hpp"
#include "utils.hpp"
#include "ray.hpp"
#include "scene.hpp"
#include "tracer.hpp"
using namespace std;

class Render {
protected:
    Scene* scene;
    V3* img;
    int w,h;
public:
    Render(int w_,int h_,Scene* scene_):w(w_),h(h_),scene(scene_) {
        img = new V3[w * h];
    }
    virtual void rendering() = 0;

    void generateIMG(const std::string& fn) {
        FILE *f = fopen(fn.c_str(),"w");
        fprintf(f, "P6\n%d %d\n%d\n", w, h, 255);
        for (int i = h - 1;i >= 0; --i) {
            for (int j = w - 1; j >= 0; --j) {
                fprintf(f, "%c%c%c", toColor(img[i * w + j][0]), toColor(img[i * w + j][1]), toColor(img[i * w + j][2]));
            }
        }
    }
    virtual ~Render() {
        delete img;
    }
};


class PT: public Render {
private:
    V3 radiance(const Ray&r, int dep,unsigned short *X, int maxdepth = 1000){
        Intersection res;
        if(!scene->findNearest_naive(r,res)) return V3();
        Object* obj = scene->getObj(res.id);

        V3 x = r.pos(res.t),n = res.n,color=obj->material.color(res.a,res.b);

        double p=color.max();
        if (dep > maxdepth) return obj->material.e;
        if(++dep > 5)
            if(erand48(X)<p) color /= p;
            else return obj->material.e;
        if(obj->material.refl == DIFF){
            double phi = 2 * PI * erand48(X), theta = erand48(X), gamma = sqrt(theta);
            V3 w = n, u = ((fabs(w[0]) > EPS? V3(0,1):V3(1)) & w).norm(), v = w & u;
            //随机一个方向
            V3 d = (u * cos(phi) * gamma + v * sin(phi) * gamma + w * sqrt(1 - theta)).norm();
            return obj->material.e + color.mult(radiance(Ray(x,d),dep,X));
        }
        else{
            Ray reflray = Ray(x,r.d.reflect(n));
            if (obj->material.refl == SPEC){
                return obj->material.e + color.mult(radiance(reflray,dep,X)); 
            }
            else{
                V3 d = r.d.refract(n, res.into?1:obj->material.ns, res.into?obj->material.ns:1); //...
                if (d.len2()<EPS) {
                    //全反射
                    return obj->material.e + color.mult(radiance(reflray, dep,X));
                }
            
                double a = obj->material.ns - 1;
                double b = obj->material.ns + 1;
                double R0 = a * a / (b * b);
                double c = 1 - (res.into? -r.d.dot(n):-d.dot(n)); 
                double Re = R0 + (1 - R0) * c * c * c * c * c;
                double Tr = 1 - Re;
                double P = 0.25 + 0.5 * Re;
                double RP = Re / P;
                double TP = Tr / (1 - P); 
                return obj->material.e + color.mult(dep>2 ? (erand48(X)<P ? 
                    radiance(reflray,dep,X)*RP:radiance(Ray(x,d),dep,X)*TP) : 
                    radiance(reflray,dep,X)*Re+radiance(Ray(x,d),dep,X)*Tr); 
            }
        }
    }

public:
    int samp;

    PT(int w_,int h_,int samp_,Scene* scene_):Render(w_,h_,scene_),samp(samp_){}


    void rendering() override {
        Ray cam(V3(70,32,280), V3(-0.15,0.05,-1).norm()); 

        double fr = 0.5;

        V3 cx = V3(w * fr / h);
        V3 cy = (cx & cam.d).norm() * fr; 
        V3 r;

        //渐进式 无景深
        /*
        for (int s = 0;s < samp; ++s) {
            #pragma omp parallel for schedule(dynamic, 1) private(r)
            for (int y = 0;y < h; ++y) {
                for (int x = 0;x < w; ++x) {
                    for (int sy = 0;sy < 2; ++sy) {
                        for (int sx = 0;sx < 2; ++sx) {
                            unsigned short X[3] = {y + sx + 1, y * x + sy, y * x * y + sx * sy};
                            double r1 = 2 * erand48(X), dx = r1 < 1? sqrt(r1): 2 - sqrt(2 - r1);
                            double r2 = 2 * erand48(X), dy = r2 < 1? sqrt(r2): 2 - sqrt(2 - r2);
                            V3 d = cx * ((sx + dx / 2 + x) / w - 0.5) + cy * ((sy + dy / 2 + y) / h - 0.5) + cam.d;
                            img [y * w + x] += radiance(Ray(cam.o + d * 120, d.norm()), 0 , X);
                        }
                    }
                }
            }
        }
        */


        #pragma omp parallel for schedule(dynamic, 1) private(r)
        for (int y = 0; y < h; ++y) {
            fprintf(stderr,"\rpath tracing  use %d samp  %5.2f%%",samp * 4,100.0 * y / h);
            for (int x = 0;x < w; ++x) {
                for (int sy = 0; sy < 2; ++sy) {
                    for (int sx = 0; sx < 2; ++sx) {
                        unsigned short X[3] = {y + sx, y * x + sy, y * y * x + sx * sy};
                        //basic
                        r[0] = r[1] = r[2] = 0;
                        #ifndef DEPTH
                        for (int s = 0;s < samp; ++s) {
                            double r1 = 2 * erand48(X), dx = r1 < 1? sqrt(r1): 2 - sqrt(2 - r1);
                            double r2 = 2 * erand48(X), dy = r2 < 1? sqrt(r2): 2 - sqrt(2 - r2);
                            V3 d = cx * ((sx + dx / 2 + x) / w - 0.5) + cy * ((sy + dy / 2 + y) / h - 0.5) + cam.d;
                            r += radiance(Ray(cam.o + d * 110, d.norm()), 0, X);

                        }
                        #else
                        //景深随机取点
                        V3 d = cx * (x * 1.0 / w - 0.5) + cy * (y * 1.0 / h - 0.5) + cam.d;
                        //与焦平面交点
                        Intersection result;
                        Ray ray(cam.o + d * 120, d.norm());
                        if (!scene->focal->intersect(ray,result)) continue;
                        V3 pos = ray.pos(result.t);
                        //随机取点
                        for (int s = 0; s < samp; ++s) {
                            double rad = FOCAL_RAD * erand48(X);
                            double theta = 2 * erand48(X) * PI;
                            V3 origin = cam.o + d * 120 + cx * rad * cos(theta) + cy * rad * sin(theta);
                            r += radiance(Ray(origin,(pos - origin).norm()),0,X);
                        }
                        #endif


                        #ifdef GOD_RAY
                        img[y * w + x] += (r / samp) / 4;
                        #else
                        img[y * w + x] += (r / samp).clamp() / 4;
                        #endif
                    }
                } 
            
            }
        }

        
        //体积光 ray march        
        #ifdef GOD_RAY
        #pragma omp parallel for schedule(dynamic, 1) 
        for (int y = 0;y < h; ++y) {
            fprintf(stderr,"\rray marching  %5.2f%%",100.*y/h);
            for (int x = 0; x < w; ++x) {
                unsigned short X[3]={y + 1 ,y * x + 2, y * x * y + 3};
                V3 d = cx * (x * 1.0 / w - 0.5) + cy * (y * 1.0 / h - 0.5) + cam.d;
                Intersection result;
                Ray ray(cam.o + d * 120, d);
                if(!scene->findNearest_naive(ray,result)) continue; //basic

                //直接计算
                const int stepNum = 100;
                //const double e = 1000000; //basic
                const double e = 400000; //basic
                //const double e = 50000; //单独计算
                double stepSize = result.t / stepNum;
                double t = 0;
                double l = 0;
                for (int k = 0;k < stepNum; ++k) {
                    V3 p = ray.pos(t);
                   

                    for (int s = 0; s < GOD_RAY_SAMP; ++s) {
                        //物理模拟
                        //Ray r(p,V3(2 * erand48(X) - 1,2 * erand48(X) - 1,2 * erand48(X) - 1));
                        
                        
                        //直接求交检查是否可见
                        V3 d(2 * erand48(X) - 1,2 * erand48(X) - 1,2 * erand48(X) - 1);
                        V3 origin = p + d / w;
                        //Ray r(p,scene->lighter->o - p);
                        Ray r(origin,d);
                        
                        Intersection tmp;

                        
                        if (scene->findNearest_naive(r,tmp)) {
                            if (tmp.id == scene->lighter->id) {
                                double vlight = e / (p - scene->lighter->o).len2();
                                //HG公式计算系数
                                double g = 0.5;
                                double costheta = (-ray.d).norm().dot((scene->lighter->o - p).norm());
                                double tmp = (1 + g * g - 2 * g * costheta);
                                double hg = (1 - g * g) / (4 * PI * pow(tmp,1.5));
                                //cout << "hg: " << hg << endl;
                                l += vlight / GOD_RAY_SAMP * hg;
                            }
                        }
                                                
                    }

                    t += stepSize;
                }
                img[y * w + x] += l;
                img[y * w + x] = img[y * w + x].clamp();
            }
        }
        #endif
        
        
    }

    
};


class PM: public Render {
private:
    V3 pm_radiance(const Ray& r,PhotonMap& pm,int dep, unsigned short *X) {

        Intersection res;
        //cout << "solve" << endl;
        if(!scene->findNearest_naive(r,res))return V3();
        //cout << "intersect" << endl;
        Object* obj = scene->getObj(res.id);
        //cout << "id: " << res.id << endl;

        V3 x = r.pos(res.t), n = res.n, color = obj->material.color(res.a,res.b);

        double p = color.max();
        if(++dep > 5)
            if(erand48(X) < p) color /= p;
            else return obj->material.e;
        if (dep > 25) return V3();
        if(obj->material.refl == DIFF){
            //cout << "into diff" << endl;
            int photon_num = 0;
            //估计
            V3 tmp =  pm.irradEstimate(x,color,PM_R,photon_num) * p;
            return obj->material.e + tmp;
        }
        else{
            Ray reflray = Ray(x,r.d.reflect(n));
            if (obj->material.refl == SPEC){
                return obj->material.e + color.mult(pm_radiance(reflray, pm, dep, X)); 
            }
            else{
                V3 d = r.d.refract(n, res.into?1:obj->material.ns, res.into?obj->material.ns:1); //...
                if (d.len2()<EPS) //全反射
                    return obj->material.e + color.mult(pm_radiance(reflray,pm, dep,X));
                double a = obj->material.ns - 1;
                double b = obj->material.ns + 1;
                double R0 = a * a / (b * b);
                double c = 1 - (res.into? -r.d.dot(n):-d.dot(n)); 
                double Re = R0 + (1 - R0) * c * c * c * c * c;
                double Tr = 1 - Re;
                double P = 0.25 + 0.5 * Re;
                double RP = Re / P;
                double TP = Tr / (1 - P); 
                return obj->material.e + color.mult(dep>2 ? (erand48(X)<P ? 
                    pm_radiance(reflray,pm,dep,X)*RP:pm_radiance(Ray(x,d),pm,dep,X)*TP) : 
                    pm_radiance(reflray,pm,dep,X)*Re + pm_radiance(Ray(x,d),pm,dep,X)*Tr); 
            }
        }
    }
public:
    int samp;
    PM(int w_,int h_,int samp_,Scene* scene_):Render(w_,h_,scene_),samp(samp_){}

    void rendering() override {
        PhotonMap pm(PM_MAX_PHOTONS);
        PMtracer pmtracer(PM_EMIT_PHOTONS,scene,&pm);
        pmtracer.run();
        cout << "run done" << endl;
        Ray cam(V3(70,32,280), V3(-0.15,0.05,-1).norm()); //basic

        double fr = 0.5; //basic

        V3 cx = V3(w * fr / h);
        V3 cy = (cx & cam.d).norm() * fr, r;


        #pragma omp parallel for schedule(dynamic, 1) private(r)
        for(int y = 0; y < h; ++y){
            fprintf(stderr,"\rpm rendering %d samp  %5.2f%%",samp * 4,100.0 * y / h);
            for(int x = 0;x < w;++x){
                
                //有明显锯齿
                /*
                unsigned short X[3]={y,y*x,y*x*y};
                V3 d=cx*(x * 1.0/w-.5)+cy*(y * 1.0/h-.5)+cam.d; 
                V3 tmp = pm_radiance(Ray(cam.o + d * 120,d.norm()),pm,0,X);
                img[y * w + x] = tmp.clamp();
                */
                
               
                for(int sy = 0;sy < 2;++sy)
                    for(int sx = 0;sx < 2;++sx)
                    {
                        unsigned short X[3]={y + sx,y * x + sy,y * x * y + sx * sy};
                        //basic
                        r[0]=r[1]=r[2]=0;
                        for(int s = 0;s < samp; ++s){
                            double r1 = 2 * erand48(X), dx=r1<1 ? sqrt(r1): 2-sqrt(2-r1);
                            double r2 = 2 * erand48(X), dy=r2<1 ? sqrt(r2): 2-sqrt(2-r2);
                            V3 d = cx * ((sx + dx / 2 + x) / w - 0.5) + cy * ((sy + dy / 2 + y) / h - 0.5) + cam.d; 
                            r += pm_radiance(Ray(cam.o + d * 120,d.norm()),pm,0,X); 
                        }
                        img[y*w+x] += (r / samp).clamp() / 4;
                    }
                
            }
        }
    }
};


#endif