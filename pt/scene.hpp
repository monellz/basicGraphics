#ifndef _SCENE_
#define _SCENE_
#include "utils.hpp"
#include "obj.hpp"
#include "sphere.hpp"
#include "plane.hpp"
#include "triangle.hpp"

class Scene {
private:
    std::vector<Object*> objs;
public:
    Scene(){
        objs.push_back(new Sphere(objs.size(),1e5, V3( 1e5+1,40.8,81.6), V3(),V3(.1,.25,.25),SPEC));//Left ??????????
        objs.push_back(new Sphere(objs.size(),1e5, V3(-1e5+99,40.8,81.6),V3(),V3(.25,.75,.25),DIFF));//Rght 
        //new Sphere(objs.size(),1e5, V3(50,40.8, 1e5),     V3(),V3(.75,.25,.25),DIFF),//Back 
        objs.push_back(new Plane(objs.size(),V3(0,0,1),0,V3(),V3(.75,.25,.25),DIFF));  //Back

        //new Sphere(3,1e5, V3(50,40.8,-1e5+170), V3(),V3(.25,.25,.25),DIFF),//Frnt 
        objs.push_back(new Sphere(objs.size(),1e5, V3(50,40.8,-1e5+500), V3(),V3(.25,.25,.25),DIFF));//Frnt 

        objs.push_back(new Sphere(objs.size(),1e5, V3(50, 1e5, 81.6),    V3(),V3(.75,.75,.75),DIFF));//Botm 

        objs.push_back(new Sphere(objs.size(),1e5, V3(50,-1e5+81.6,81.6),V3(),V3(.75,.75,.75),DIFF));//Top 

        //-----------------------//

	    //pos 左右(大的在左),上下(大的在上),前后(大的在前)
        objs.push_back(new Sphere(objs.size(),16.5,V3(27,16.5,47),       V3(),V3(1,1,1)*.999, SPEC));//Mirr 
        //new Sphere(6,16.5,V3(27,16.5,47),       V3(),"floor.bmp", SPEC),//Mirr 
        //new Sphere(6,16.5,V3(27,16.5,47),       V3(),V3(1,1,1)*.999, REFR),//Mirr 


        objs.push_back(new Sphere(objs.size(),16.5,V3(73,16.5,78),       V3(),V3(1,1,1)*.999, REFR));//Glas 
        //new Sphere(7,16.5,V3(73,16.5,78),       V3(),V3(1,1,1)*.999, SPEC),//Glas 

        //new Sphere(8,16.5,V3(20,60,100),       V3(),V3(.25,.25,.75), DIFF),//Glas 
        objs.push_back(new Sphere(objs.size(),16.5,V3(20,60,100),       V3(),"marble.bmp", DIFF));//Glas 

        objs.push_back(new Sphere(objs.size(),600, V3(50,681.6-.27,81.6),V3(12,12,12),  V3(), DIFF)); //Lite 
        //objs.push_back(new Sphere(objs.size(),600, V3(50,681.6-2.5,101.6),V3(12,12,12),  V3(), DIFF)); //Lite 
        //new Sphere(9,600, V3(50,681.6-.27,81.6),V3(12,12,12) * 10,  V3(), DIFF) //Lite 



        //--------------obj-----------------
        objs.push_back(new Triangle(objs.size(),V3(20,60,80),V3(20,40,80),V3(40,40,80),V3(0,0,1),V3(),V3(.25,.25,.95),DIFF));
    }

    bool findNearest_naive(const Ray& r, double& t, Intersection& res) {
        double inf = t = 1e30, tmp;
        for (int i = 0;i < objs.size(); ++i) {
            Intersection foo;
            if ((tmp = objs[i]->intersect(r,foo)) && tmp < t) {
                res = foo;
                t = tmp;
            }
        }
        return t < inf;
    }

    Object* getObj(int id) {
        return objs[id];
    }

    ~Scene() {
        for (int i = 0;i < objs.size(); ++i) delete objs[i];
        objs.clear();
    }
};



#endif