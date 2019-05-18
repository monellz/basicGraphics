#include "v3.hpp"
#include "utils.hpp"
#include "ray.hpp"
#include "scene.hpp"
#include "camera.hpp"

Scene scene;

const int DEPTH = 5; //basic
//const int DEPTH = 2; //basic

V3 radiance(const Ray&r, int dep,unsigned short *X){
	Intersection res;
	//if(!intersect(r,t,id))return V3();
	//if(!intersect(r,t,res))return V3();
	if(!scene.findNearest_naive(r,res))return V3();
	//Sphere&obj=scene[id];
	Object* obj = scene.getObj(res.id);
	//if (res.into == 0 && res.id == 7) cout << res.id << endl;

	//V3 x=r.pos(t),n=(x-obj.o).norm(),f=obj.material.c,nl=n.dot(r.d)<0?into=1,n:-n;
	V3 x=r.pos(res.t),nl = res.n,f=obj->material.color(res.a,res.b);
	//n 球心到交点
	//nl 入射光对应法向量

	double p=f.max();
	if(++dep> DEPTH)
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

int main(int argc, char*argv[])
{
	int w=atoi(argv[1]), h=atoi(argv[2]), samp=atoi(argv[4])/4;
	std::string fn(argv[3]);
	

	//Ray cam(V3(70,32,280), V3(-0.15,0.05,-1).norm()); //basic
	Ray cam(V3(30,32,280), V3(0.15,-0.05,-1).norm()); //basic

	double fr = 0.5; //basic

	V3 cx=V3(w*fr/h), cy=(cx&cam.d).norm()* fr, r, *c=new V3[w*h];
	//Camera cam(w,h,V3(70,32,280),V3(-0.15,0.05,-1).norm());


#pragma omp parallel for schedule(dynamic, 1) private(r)
	for(int y=0;y<h;++y){
		fprintf(stderr,"\rUsing %d spp  %5.2f%%",samp*4,100.*y/h);
		for(int x=0;x<w;++x){
			for(int sy=0;sy<2;++sy)
				for(int sx=0;sx<2;++sx)
				{
					unsigned short X[3]={y+sx,y*x+sy,y*x*y+sx*sy};
					r[0]=r[1]=r[2]=0;
					for(int s=0;s<samp;++s){
						double r1=2*erand48(X), dx=r1<1 ? sqrt(r1): 2-sqrt(2-r1);
						double r2=2*erand48(X), dy=r2<1 ? sqrt(r2): 2-sqrt(2-r2);
						V3 d=cx*((sx+dx/2+x)/w-.5)+cy*((sy+dy/2+y)/h-.5)+cam.d; 
						r+=radiance(Ray(cam.o+d*120,d.norm()),0,X);
					}
					c[y*w+x]+=(r/samp).clamp()/4;
				}
		}
	}
	FILE*f=fopen(argv[3],"w");
	fprintf(f,"P6\n%d %d\n%d\n", w,h,255);
	for(int y=h-1;y>=0;--y)
		for(int x=w-1;x>=0;--x)
			fprintf(f,"%c%c%c",toColor(c[y*w+x][0]),toColor(c[y*w+x][1]),toColor(c[y*w+x][2]));
	return 0;
}
