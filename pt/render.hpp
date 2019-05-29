#ifndef _RENDER_
#define _RENDER_

#include "v3.hpp"
#include "utils.hpp"
#include "ray.hpp"
#include "scene.hpp"
#include "camera.hpp"
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
    V3 radiance(const Ray&r, int dep,unsigned short *X){
        Intersection res;
        //if(!intersect(r,t,id))return V3();
        //if(!intersect(r,t,res))return V3();
        if(!scene->findNearest_naive(r,res))return V3();
        //Sphere&obj=scene[id];
        Object* obj = scene->getObj(res.id);
        //if (res.into == 0 && res.id == 7) cout << res.id << endl;

        //V3 x=r.pos(t),n=(x-obj.o).norm(),f=obj.material.c,nl=n.dot(r.d)<0?into=1,n:-n;
        V3 x=r.pos(res.t),nl = res.n,f=obj->material.color(res.a,res.b);
        //n 球心到交点
        //nl 入射光对应法向量

        double p=f.max();
        if(++dep> 5)
            if(erand48(X)<p) f/=p;
            //if(erand48(X)<p) f = f;
            else return obj->material.e;
            //else return obj.material.c;
        if(obj->material.refl==DIFF){
            double r1=2*PI*erand48(X), r2=erand48(X), r2s=sqrt(r2);
            V3 w=nl, u=((fabs(w[0])>.1?V3(0,1):V3(1))&w).norm(), v=w&u;
            V3 d = (u*cos(r1)*r2s + v*sin(r1)*r2s + w*sqrt(1-r2)).norm();
            return obj->material.e + f.mult(radiance(Ray(x,d),dep,X));
        }
        else{
            Ray reflray = Ray(x,r.d.reflect(nl));
            if (obj->material.refl == SPEC){
                return obj->material.e + f.mult(radiance(reflray,dep,X)); 
            }
            else{
                //V3 d = r.d.refract(n, into?1:obj.ns, into?obj.ns:1); //...
                V3 d = r.d.refract(nl, res.into?1:obj->material.ns, res.into?obj->material.ns:1); //...
                if (d.len2()<EPS) // Total internal reflection 
                    return obj->material.e + f.mult(radiance(reflray, dep,X));
                //c = 1 - cos(theta(i))
                //double a=obj.ns-1, b=obj.ns+1, R0=a*a/(b*b), c = 1-(into?-r.d.dot(nl):d.dot(n)); 
                double a=obj->material.ns-1, b=obj->material.ns+1, R0=a*a/(b*b), c = 1-(res.into?-r.d.dot(nl):-d.dot(nl)); 
                double Re=R0+(1-R0)*c*c*c*c*c,Tr=1-Re,P=.25+.5*Re,RP=Re/P,TP=Tr/(1-P); 
                return obj->material.e + f.mult(dep>2 ? (erand48(X)<P ?   // Russian roulette 
                    radiance(reflray,dep,X)*RP:radiance(Ray(x,d),dep,X)*TP) : 
                    radiance(reflray,dep,X)*Re+radiance(Ray(x,d),dep,X)*Tr); 
            }
        }
    }

public:
    int samp;

    PT(int w_,int h_,int samp_,Scene* scene_):Render(w_,h_,scene_),samp(samp_){}


    void rendering() override {
        Ray cam(V3(70,32,280), V3(-0.15,0.05,-1).norm()); //basic
        //Ray cam(V3(-5,32,280), V3(0.15,-0.05,-1).norm()); 
        //Ray cam(V3(-5,50,280), V3(0.15,-0.15,-1).norm()); 

        double fr = 0.5; //basic

        V3 cx=V3(w*fr/h), cy=(cx&cam.d).norm()* fr, r;
        //Camera cam(w,h,V3(70,32,280),V3(-0.15,0.05,-1).norm());


        #pragma omp parallel for schedule(dynamic, 1) private(r)
        for(int y=0;y<h;++y){
            fprintf(stderr,"\rUsing %d spp  %5.2f%%",samp*4,100.*y/h);
            for(int x=0;x<w;++x){
                for(int sy=0;sy<2;++sy)
                    for(int sx=0;sx<2;++sx)
                    {
                        unsigned short X[3]={y+sx,y*x+sy,y*x*y+sx*sy};
                        //basic
                        r[0]=r[1]=r[2]=0;
                        for(int s=0;s<samp;++s){
                            double r1=2*erand48(X), dx=r1<1 ? sqrt(r1): 2-sqrt(2-r1);
                            double r2=2*erand48(X), dy=r2<1 ? sqrt(r2): 2-sqrt(2-r2);
                            V3 d=cx*((sx+dx/2+x)/w-.5)+cy*((sy+dy/2+y)/h-.5)+cam.d; 
                            r+=radiance(Ray(cam.o+d*120,d.norm()),0,X); //basic
                        }
                        img[y*w+x]+=(r/samp).clamp()/4;
                    }
            
            
            }
        }
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

        V3 x=r.pos(res.t),nl = res.n,f=obj->material.color(res.a,res.b);
        //n 球心到交点
        //nl 入射光对应法向量

        double p=f.max();
        if(++dep> 5)
            if(erand48(X)<p) f/=p;
            //if(erand48(X)<p) f = f;
            //else return obj->material.e;
            else return obj->material.e;
        if(obj->material.refl==DIFF){
            //cout << "into diff" << endl;
            int photon_num = 0;
            V3 tmp =  pm.irradEstimate(x,f,10,photon_num) * p;
            //V3 tmp =  pm.irradEstimate(x,f,5,photon_num) * p;
            //tmp.print();
            return obj->material.e + tmp;
            //return pm.irradEstimate(x,res.n,f,5,100000);

        }
        else{
            Ray reflray = Ray(x,r.d.reflect(nl));
            if (obj->material.refl == SPEC){
                return obj->material.e + f.mult(pm_radiance(reflray, pm, dep, X)); 
            }
            else{
                //V3 d = r.d.refract(n, into?1:obj.ns, into?obj.ns:1); //...
                V3 d = r.d.refract(nl, res.into?1:obj->material.ns, res.into?obj->material.ns:1); //...
                if (d.len2()<EPS) // Total internal reflection 
                    return obj->material.e + f.mult(pm_radiance(reflray,pm, dep,X));
                //c = 1 - cos(theta(i))
                //double a=obj.ns-1, b=obj.ns+1, R0=a*a/(b*b), c = 1-(into?-r.d.dot(nl):d.dot(n)); 
                double a=obj->material.ns-1, b=obj->material.ns+1, R0=a*a/(b*b), c = 1-(res.into?-r.d.dot(nl):-d.dot(nl)); 
                double Re=R0+(1-R0)*c*c*c*c*c,Tr=1-Re,P=.25+.5*Re,RP=Re/P,TP=Tr/(1-P); 
                return obj->material.e + f.mult(dep>2 ? (erand48(X)<P ?   // Russian roulette 
                    pm_radiance(reflray,pm,dep,X)*RP:pm_radiance(Ray(x,d),pm,dep,X)*TP) : 
                    pm_radiance(reflray,pm,dep,X)*Re+pm_radiance(Ray(x,d),pm,dep,X)*Tr); 
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
        //Ray cam(V3(-5,32,280), V3(0.15,-0.05,-1).norm()); 
        //Ray cam(V3(-5,50,280), V3(0.15,-0.15,-1).norm()); 

        double fr = 0.5; //basic

        V3 cx=V3(w*fr/h), cy=(cx&cam.d).norm()* fr, r;
        //Camera cam(w,h,V3(70,32,280),V3(-0.15,0.05,-1).norm());


        #pragma omp parallel for schedule(dynamic, 1) private(r)
        for(int y=0;y<h;++y){
            fprintf(stderr,"\rUsing %d spp  %5.2f%%",samp*4,100.*y/h);
            for(int x=0;x<w;++x){
                
                //有明显锯齿
                /*
                unsigned short X[3]={y,y*x,y*x*y};
                V3 d=cx*(x * 1.0/w-.5)+cy*(y * 1.0/h-.5)+cam.d; 
                V3 tmp = pm_radiance(Ray(cam.o + d * 120,d.norm()),pm,0,X);
                img[y * w + x] = tmp.clamp();
                */
                
               
                for(int sy=0;sy<2;++sy)
                    for(int sx=0;sx<2;++sx)
                    {
                        unsigned short X[3]={y+sx,y*x+sy,y*x*y+sx*sy};
                        //basic
                        r[0]=r[1]=r[2]=0;
                        for(int s=0;s<samp;++s){
                            double r1=2*erand48(X), dx=r1<1 ? sqrt(r1): 2-sqrt(2-r1);
                            double r2=2*erand48(X), dy=r2<1 ? sqrt(r2): 2-sqrt(2-r2);
                            V3 d=cx*((sx+dx/2+x)/w-.5)+cy*((sy+dy/2+y)/h-.5)+cam.d; 
                            //r+=radiance(Ray(cam.o+d*120,d.norm()),0,X); //basic
                            r+=pm_radiance(Ray(cam.o+d*120,d.norm()),pm,0,X); 
                        }
                        img[y*w+x]+=(r/samp).clamp()/4;
                    }
                
            }
        }
    }
};


#endif