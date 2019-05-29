#ifndef _SCENE_
#define _SCENE_
#include "utils.hpp"
#include "obj.hpp"
#include "sphere.hpp"
#include "plane.hpp"
#include "triangle.hpp"
#include "kdtree.hpp"

class Scene {
public:
    std::vector<Object*> objs;
    kdTree* tree;
    Sphere* lighter;
    Scene(){
        
        //objs.push_back(new Sphere(objs.size(),1e5, V3( 1e5+1,40.8,81.6), V3(),V3(.75,.25,.25),SPEC));//Left ??????????
        //objs.push_back(new Sphere(objs.size(),1e5, V3(-1e5+99,40.8,81.6),V3(),V3(.25,.25,.75),DIFF));//Rght 
        
        //new Sphere(objs.size(),1e5, V3(50,40.8, 1e5),     V3(),V3(.75,.25,.25),DIFF),//Back 

        objs.push_back(new Plane(objs.size(),V3(0,0,1),-1,V3(),V3(.25,.25,.75),DIFF));  //Back
        //objs.push_back(new Plane(objs.size(),V3(0,0,1),0,V3(),"floor.bmp",DIFF));  //Back

        //new Sphere(3,1e5, V3(50,40.8,-1e5+170), V3(),V3(.25,.25,.25),DIFF),//Frnt 
        //objs.push_back(new Sphere(objs.size(),1e5, V3(50,40.8,-1e5+500), V3(),V3(.25,.25,.25),DIFF));//Frnt 
        
        //objs.push_back(new Sphere(objs.size(),1e5, V3(50, 1e5, 81.6),    V3(),V3(.75,.75,.75),DIFF));//Botm 
        //objs.push_back(new Sphere(objs.size(),1e5, V3(50, 1e5, 81.6),    V3(),"floor.bmp",DIFF));//Botm 
        objs.push_back(new Plane(objs.size(), V3(0,1,0),0,V3(), "floor.bmp", DIFF));

        
        //objs.push_back(new Sphere(objs.size(),1e5, V3(50,-1e5+81.6,81.6),V3(),V3(.75,.75,.75),DIFF));//Top 

        //-----------------------//

	    //pos 左右(大的在左),上下(大的在上),前后(大的在前)
        objs.push_back(new Sphere(objs.size(),16.5,V3(27,16.5,47),       V3(),V3(1,1,1)*.999, SPEC));//Mirr 
        //new Sphere(6,16.5,V3(27,16.5,47),       V3(),"floor.bmp", SPEC),//Mirr 
        //new Sphere(6,16.5,V3(27,16.5,47),       V3(),V3(1,1,1)*.999, REFR),//Mirr 

        //new Sphere(8,16.5,V3(20,60,100),       V3(),V3(.25,.25,.75), DIFF),//Glas 
        objs.push_back(new Sphere(objs.size(),16.5,V3(20,40,60),       V3(),"marble.bmp", DIFF));//Glas 
        objs.push_back(new Sphere(objs.size(),16.5,V3(73,16.5,78),       V3(), V3(1,1,1) * 0.999, REFR));//Glas 


        

        //lighter =new Sphere(objs.size(),30, V3(40,131.6-.27,81.6),V3(12,12,12),  V3(), DIFF); //Lite 
        lighter =new Sphere(objs.size(),30, V3(40,131.6-.27,81.6),V3(12,12,12),  V3(), DIFF); //Lite 
        objs.push_back(lighter);
        //objs.push_back(new Sphere(objs.size(),30, V3(40,131.6-.27,81.6),V3(12,12,12),  V3(), DIFF)); //Lite 
        //objs.push_back(new Sphere(objs.size(),30, V3(50,101.6-.5,101.6),V3(12,12,12),  V3(), DIFF)); //Lite 
        //new Sphere(9,600, V3(50,681.6-.27,81.6),V3(12,12,12) * 10,  V3(), DIFF) //Lite 

        

        
        //--------------obj-----------------
        //objs.push_back(new Triangle(objs.size(),V3(20,60,80),V3(20,40,80),V3(40,40,80),V3(0,0,1),V3(),V3(.25,.25,.95),DIFF));
        //readObj("love.obj");


        std::cout << "obj total: " << objs.size() << std::endl;
        std::cout << "build tree" << std::endl;
        tree = new kdTree(objs);
        std::cout << "build tree done" << std::endl;
    }

    void readObj(std::string fn) {
        //读取obj文件并放入objs中
        std::cout << "reading obj..." << std::endl;
        ifstream ifs(fn);
        std::vector<V3> buf;
        std::string s;
        while (getline(ifs,s)) {
            if (s.length() < 2) continue;
            //std::cout << s << std::endl;
            if (s[0] == 'v') {
                //点
                double x,y,z;
                std::string type;
                istringstream in(s);
                in >> type >> x >> y >> z;
                buf.push_back(V3(x,y,z));
            } else if (s[0] == 'f') {
                //面
                int i,j,k;
                istringstream in(s);
                std::string type;
                in >> type >> i >> j >> k;
                objs.push_back(new Triangle(objs.size(),buf[i - 1],buf[j - 1],buf[k - 1],(buf[i - 1] - buf[j - 1]) & (buf[j - 1] - buf[k - 1]), V3(),V3(.85,.25,.25), REFR));        
                //if (objs.size() > 500) break;
            } else {
                std::cout << s << endl;
            }
        }
        std::cout << "read done ..." << std::endl;
    }


    bool findNearest_naive(const Ray& r, Intersection& res) {
        /*
        double t = 1e30;
        double inf = 1e30;
        for (int i = 0;i < objs.size(); ++i) {
            Intersection foo;
            if (objs[i]->intersect(r,foo) && foo.t < t) {
                res = foo;
                t = foo.t;
            }
        }
        bool ok =  t < inf;

        Intersection repeat;
        bool test = tree->intersect(r,repeat,objs);
        */
        return tree->intersect(r,res,objs);
    }

    Object* getObj(int id) {
        return objs[id];
    }

    Photon emitPhoton(int cnt) {
        Photon pt;
        Ray r = lighter->light();
        pt.pos = r.o;
        pt.dir = r.d;
        pt.power = lighter->material.e * 4 * PI * PI * lighter->rad * lighter->rad / cnt;
        //pt.power = lighter->material.e * 4 * PI * lighter->rad * lighter->rad / cnt;
        return pt;
    }

    ~Scene() {
        for (int i = 0;i < objs.size(); ++i) delete objs[i];
        objs.clear();
    }
};



#endif