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
    double shadowmap[SHADOW_SPLIT_NUM][SHADOW_SPLIT_NUM];

    double checkDepth(double theta, double fai) {
        //返回这一点的深度
        //二分搜索

        double dtheta = PI / SHADOW_SPLIT_NUM;
        double dfai = 2 * PI / SHADOW_SPLIT_NUM;

        int lo = 0, mid;
        int half, len = SHADOW_SPLIT_NUM;
        while (len > 0) {
            half = len >> 1;
            mid = lo + half;
            if (mid * dtheta < theta) {
                lo = mid + 1;
                len = len - half - 1; //在右边序列中查找
            } else len = half; //左边序列
        }

        int i = lo;

        lo = 0;
        len = SHADOW_SPLIT_NUM;
        while (len > 0) {
            half = len >> 1;
            mid = lo + half;
            if (mid * dfai < fai) {
                lo = mid + 1;
                len = len - half - 1;
            } else len = half;
        }

        int j = lo;

        return shadowmap[i][j]; //basic
        //return shadowmap[i + 1][j + 1];
    }

    void createShadowMap() {
        /*
        只考虑球光源
        方向(theta, fai) 到深度的map

        坐标转换
        (x,y,z) = (sin theta * cos fai, sin theta * sin fai, cos theta)
        (theta, fai) = (acos(z), atan(y / x))

        theta \in [0, pi] 
        fai \in [0, 2pi]

        将theta,fai均分
        */

        cout << "create shadow map" << endl;
        double dtheta = PI / SHADOW_SPLIT_NUM;
        double dfai = 2 * PI / SHADOW_SPLIT_NUM;

        //预处理sin cos
        double cosk[SHADOW_SPLIT_NUM];
        double sink[SHADOW_SPLIT_NUM];
        double cos2k[SHADOW_SPLIT_NUM];
        double sin2k[SHADOW_SPLIT_NUM];
        for (int i = 0;i < SHADOW_SPLIT_NUM; ++i) {
            cosk[i] = cos(i * dtheta);
            sink[i] = sin(i * dtheta);
            cos2k[i] = cos(i * dfai);
            sin2k[i] = sin(i * dfai);
        }
        for (int i = 0;i < SHADOW_SPLIT_NUM; ++i) {
            for (int j = 0;j < SHADOW_SPLIT_NUM; ++j) {
                //测试深度
                V3 d(sink[i] * cos2k[j], sink[i] * sin2k[j], cosk[i]);
                //V3 d(sin(i * dtheta) * cos(j * dfai), sin(i * dtheta) * sin(j * dfai), cos(i * dtheta));
                //V3 o = scene->lighter->o + d * (scene->lighter->rad + EPS);
                V3 o = scene->lighter->o + d * (scene->lighter->rad);
                Ray r(o,d);

                Intersection res;
                if (scene->findNearest_naive(r,res)) {
                    //计算光源到碰撞点的深度
                    shadowmap[i][j] = (r.pos(res.t) - scene->lighter->o).len();
                } else {
                    //没有交点 深度无穷
                    shadowmap[i][j] = INF;
                }
            }
        }
               
    }


    V3 radiance(const Ray&r, int dep,unsigned short *X, int maxdepth = 1000){
        Intersection res;
        if(!scene->findNearest_naive(r,res))return V3();
        Object* obj = scene->getObj(res.id);

        //体积光 ray march



        V3 x=r.pos(res.t),nl = res.n,f=obj->material.color(res.a,res.b);
        //n 球心到交点
        //nl 入射光对应法向量

        double p=f.max();
        if (dep > maxdepth) return obj->material.e;
        if(++dep > 5)
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
        //Ray cam(V3(70,32,180), V3(-0.15,0.05,-1).norm()); 
        //Ray cam(V3(-170,6,80), V3(2.55,0.05,-1).norm()); 
        //Ray cam(V3(70,32,280), V3(-0.6,0.05,-1).norm()); 

        double fr = 0.5; //basic

        V3 cx=V3(w*fr/h), cy=(cx&cam.d).norm()* fr, r; //basic
        //Camera cam(w,h,V3(70,32,280),V3(-0.15,0.05,-1).norm());


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
            fprintf(stderr,"\rpath tracing  use %d samp  %5.2f%%",samp*4,100.*y/h);
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
                            V3 d = cx * ((sx + dx / 2 + x) / w - 0.5) + cy * ((sy + dy / 2 + y) / h - 0.5) + cam.d; //basic
                            //V3 d = cx * ((sx + dx / 2 + x) / w ) + cy * ((sy + dy / 2 + y) / h - 0.5) + cam.d;
                            //V3 d=cx*((sx+dx/2+x)/w-.5)+cy*((sy+dy/2+y)/h-.5)+cam.d; 
                            //r+=radiance(Ray(cam.o+d*120,d.norm()),0,X);
                            //r += radiance(Ray(cam.o + d * 120, d.norm()), 0, X);
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
                const double e = 50000;
                double stepSize = result.t / stepNum;
                double t = 0;
                double l = 0;
                for (int k = 0;k < stepNum; ++k) {
                    V3 p = ray.pos(t);
                   

                    for (int s = 0; s < GOD_RAY_SAMP; ++s) {
                        //直接求交检查是否可见
                        //Ray r(p,V3(2 * erand48(X) - 1,2 * erand48(X) - 1,2 * erand48(X) - 1));
                        V3 d(2 * erand48(X) - 1,2 * erand48(X) - 1,2 * erand48(X) - 1);
                        V3 origin = p + d / w;
                        //Ray r(p,scene->lighter->o - p);
                        Ray r(origin,scene->lighter->o - origin);
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
                        
                        //使用radiance计算
                        /*
                        Ray r(p,scene->lighter->o - p);
                        V3 radi = radiance(r,0,X,2);
                        if (radi.len2() > EPS) {
                            double vlight = e / (p - scene->lighter->o).len2();
                            //HG公式计算系数
                            double g = 0.5;
                            double costheta = (-ray.d).norm().dot((scene->lighter->o - p).norm());
                            double tmp = (1 + g * g - 2 * g * costheta);
                            double hg = (1 - g * g) / (4 * PI * pow(tmp,1.5));
                            //cout << "hg: " << hg << endl;
                            l += vlight / 10 * hg;
                        }
                        
                        */

                        /*
                        //使用shadow map检查可见
                        //计算深度
                        if (erand48(X) > 1.0 / 10) continue;
                        double depth = (p - scene->lighter->o).len();

                        //坐标转换(theta, fai) = (acos(z), atan(y / x))
                        double theta = acos(ray.d.z);
                        double fai = atan(ray.d.y / ray.d.x); //??除以0?

                        //double real_depth = checkDepth(theta,fai);
                        //cout << "depth: " << depth << " real_depth: " << real_depth << endl;
                        if (depth < checkDepth(theta,fai)) {
                            //可见
                            double vlight = e / (p - scene->lighter->o).len2();
                            //HG公式计算系数
                            double g = 0.5;
                            double costheta = (-ray.d).norm().dot((scene->lighter->o - p).norm());
                            double tmp = (1 + g * g - 2 * g * costheta);
                            double hg = (1 - g * g) / (4 * PI * pow(tmp,1.5));
                            //cout << "hg: " << hg << endl;
                            //l += vlight / 10 * hg;
                            l += vlight / 10 * hg * stepSize;
                        }
                        //否则不可见
                        */
                        
                    }

                    /*
                    //使用shadow map检查可见
                    //计算深度
                    double depth = (p - scene->lighter->o).len();

                    //坐标转换(theta, fai) = (acos(z), atan(y / x))
                    if (x == 667 || x == 666) cout << "ray.d.z: " << ray.d.z << "   y/x: " << ray.d.y / ray.d.x << endl;
                    double theta = acos(ray.d.z);
                    double fai = atan(ray.d.y / ray.d.x); //??除以0?

                    double real_depth = checkDepth(theta,fai);
                    if (depth < checkDepth(theta,fai)) {
                        //可见
                        double vlight = e / (p - scene->lighter->o).len2();
                        //HG公式计算系数
                        double g = 0.5;
                        double costheta = (-ray.d).norm().dot((scene->lighter->o - p).norm());
                        double tmp = (1 + g * g - 2 * g * costheta);
                        double hg = (1 - g * g) / (4 * PI * pow(tmp,1.5));
                        //cout << "hg: " << hg << endl;
                        //l += vlight / 10 * hg;
                        l += vlight * hg * stepSize;
                    }
                    //否则不可见

                    if (x == 667 || x == 666) {
                        cout << "x: " << x << " depth: " << depth << " real_depth: " << real_depth << endl;
                        cout <<  "    l: " << l << endl;
                    }
                    */

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
            V3 tmp =  pm.irradEstimate(x,f,PM_R,photon_num) * p;
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
            fprintf(stderr,"\rpm rendering %d samp  %5.2f%%",samp*4,100.*y/h);
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