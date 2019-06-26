#ifndef _SCENE_
#define _SCENE_
#include "utils.hpp"
#include "obj.hpp"
#include "sphere.hpp"
#include "plane.hpp"
#include "triangle.hpp"
#include "kdtree.hpp"
#include "bezier.hpp"
#include "objLoader.hpp"
#include "cube.hpp"

class Scene {
public:
    std::vector<Object*> objs;
    ObjLoader loader;

    kdTree* tree;
    Sphere* lighter;
    Plane* focal; //焦平面
    Scene(){
        srand(time(0));
        //id = -1 只能单独测试相交
        focal = new Plane(-1,V3(0,0,1),FOCAL_DIS,V3(),V3(),DIFF);
        


        
        //ppm场景                
        /*
        //objs.push_back(new Plane(objs.size(),V3(0,0,1),-1,V3(),V3(.25,.25,.75),DIFF));  //Back
        //objs.push_back(new Plane(objs.size(),V3(0,0,1),-50,V3(),V3(.25,.25,.75),DIFF));  //Back

        objs.push_back(new Plane(objs.size(),V3(0,0,1),-50,V3(),"bg.jpg",DIFF));  //Back

        //objs.push_back(new Plane(objs.size(),V3(0,0,1),-50,V3(),"sky.jpg",DIFF));  //Back
        //objs.push_back(new Plane(objs.size(),V3(1,0,0),-10,V3(),V3(.55,.25,.25),DIFF));  //Back
        //objs.push_back(new Sphere(objs.size(),1e5, V3(-1e5+99,40.8,81.6),V3(),V3(.25,.25,.75),DIFF));//Rght

        objs.push_back(new Plane(objs.size(), V3(0,1,0),0,V3(), "floor.bmp", DIFF));
	    //pos 左右(大的在左),上下(大的在上),前后(大的在前)

        objs.push_back(new Cube(objs.size(), V3(5,0,10),V3(35,10,30),V3(),V3(1,1,1) * 0.5,REFR));
        objs.push_back(new Sphere(objs.size(),13,V3(55,13,47),       V3(),V3(1,1,1)*.999, SPEC));//Mirr 
        //objs.push_back(new Sphere(objs.size(),16.5,V3(20,40,60),       V3(),"marble.bmp", DIFF));//Glas 
        objs.push_back(new Sphere(objs.size(),16.5,V3(73,16.5,78),       V3(), V3(1,1,1) * 0.999, REFR));//Glas 
        objs.push_back(new Sphere(objs.size(),10,V3(60,10,90),       V3(), V3(1,1,1) * 0.999, REFR));//Glas 
        lighter =new Sphere(objs.size(),30, V3(40,131.6-.27,81.6),V3(12,12,12),  V3(), DIFF); //Lite 
        //lighter =new Sphere(objs.size(),30, V3(25,131.6-0.27,81.6),V3(12,12,12),  V3(), DIFF); //Lite  for glass
        objs.push_back(lighter);
        //readObj("glass.obj",V3(45,0,60));
        
        //bezier
        double x[] = {-15,-7,-40,-25,25,40,7,15};
        double y[] = {-15,-50,-55,-60,-60,-55,-50,-15};
        for (int i = 0;i < 8; ++i) y[i] += 55;
        objs.push_back(new Bezier(objs.size(),x,y,8,V3(0,0,0),"vase.png",DIFF));
        loader.load("veliero.obj",objs,V3(20,10,20),REFR);
        */

        //pt 景深
        /*
        objs.push_back(new Plane(objs.size(),V3(0,0,1),-1,V3(),V3(.25,.25,.75),DIFF));  //Back
        objs.push_back(new Plane(objs.size(), V3(0,1,0),0,V3(), "floor.bmp", DIFF));
        //objs.push_back(new Sphere(objs.size(),13,V3(70,13,47),       V3(),V3(1,1,1)*.999, SPEC));//Mirr 
        objs.push_back(new Sphere(objs.size(),10,V3(-20,10,60),       V3(),V3(1,1,1)*.999, REFR));//Glas
        lighter =new Sphere(objs.size(),30, V3(40,131.6-.27,81.6),V3(12,12,12),  V3(), DIFF); //Lite 
        objs.push_back(lighter);
        

        //loader.load("veliero-rotate.obj",objs,V3(20,10,50),REFR);
        loader.load("car.obj",objs,V3(20,0,50),REFR);
        */
        
        //pt 景深2        
        objs.push_back(new Plane(objs.size(),V3(0,0,1),-1,V3(),V3(.25,.25,.75),DIFF));  //Back
        objs.push_back(new Plane(objs.size(), V3(0,1,0),0,V3(), "floor.bmp", DIFF));
        objs.push_back(new Sphere(objs.size(),15,V3(70,15,50),       V3(),V3(1,1,1)*.999, SPEC));//Mirr 
        objs.push_back(new Sphere(objs.size(),13,V3(30,13,60),       V3(),"marble.bmp", DIFF));//Mirr 
        objs.push_back(new Sphere(objs.size(),15,V3(60,15,75),       V3(), V3(1,1,1) * 0.999, REFR));//Glas 
        lighter =new Sphere(objs.size(),30, V3(40,130.4,81.6),V3(12,12,12),  V3(), DIFF); //Lite 
        objs.push_back(lighter);
        objs.push_back(new Cube(objs.size(), V3(-10,0,20),V3(10,10,80),V3(),V3(1,1,1) * 0.5,REFR));
        objs.push_back(new Sphere(objs.size(),15,V3(0,15 + 10,65),       V3(), V3(1,1,1) * 0.999, REFR));//Glas 



        //原始       
        /*
        objs.push_back(new Sphere(objs.size(),1e5, V3( 1e5+1,40.8,81.6), V3(),V3(.75,.25,.25),DIFF));//Left
        //objs.push_back(new Plane(objs.size(),V3(0,0,1),-1,V3(),V3(.25,.25,.75),DIFF));  //Back
        objs.push_back(new Sphere(objs.size(),1e5, V3(-1e5+99,40.8,81.6),V3(),V3(.25,.25,.75),DIFF));//Rght
        objs.push_back(new Sphere(objs.size(),1e5, V3(50,40.8, 1e5),     V3(),V3(.75,.75,.75),DIFF));//Back
        //objs.push_back(new Sphere(objs.size(),1e5, V3(50,40.8,-1e5+170), V3(),V3(),           DIFF));//Frnt
        objs.push_back(new Sphere(objs.size(),1e5, V3(50, 1e5, 81.6),    V3(),V3(.75,.75,.75),DIFF));//Botm
        objs.push_back(new Sphere(objs.size(),1e5, V3(50,-1e5+81.6,81.6),V3(),V3(.75,.75,.75),DIFF));//Top
        //objs.push_back(new Sphere(objs.size(),16.5,V3(27,16.5,47),       V3(),V3(1,1,1)*.999, SPEC));//Mirr
        //objs.push_back(new Sphere(objs.size(),16.5,V3(73,16.5,78),       V3(),V3(1,1,1)*.999, REFR));//Glas
        //lighter = new Sphere(objs.size(), 600, V3(50,681.6-.27,81.6),V3(1,1,1) * 12,  V3(), DIFF);//Lite
        lighter = new Sphere(objs.size(), 600, V3(50,681.6-.27,81.6),V3(1,1,1) * 12,  V3(), DIFF);//Lite
        objs.push_back(lighter);
        */
        
        //pt 体积光
        /*
        objs.push_back(new Plane(objs.size(),V3(0,0,1),-10,V3(),V3(.25,.25,.75),DIFF));  //Back
        objs.push_back(new Plane(objs.size(), V3(0,1,0),0,V3(), "floor.bmp", DIFF)); //botm

        objs.push_back(new Sphere(objs.size(),1e5, V3( 1e5+1,40.8,81.6), V3(),V3(.75,.25,.25),DIFF));//Left
        //objs.push_back(new Plane(objs.size(), V3(1,0,0),0,V3(), V3(0.75, 0.25, 0.25), DIFF)); //left
        objs.push_back(new Plane(objs.size(), V3(1,0,0),100,V3(), V3(0.25, 0.25, 0.75), DIFF)); //right
        //objs.push_back(new Sphere(objs.size(),1e5, V3(-1e5+130,40.8,81.6),V3(),V3(.25,.25,.75),DIFF));//Rght
        //objs.push_back(new Sphere(objs.size(),1e5, V3(50,40.8, 1e5),     V3(),V3(.75,.75,.75),DIFF));//Back
        //objs.push_back(new Sphere(objs.size(),1e5, V3(50,40.8,-1e5+170), V3(),V3(),           DIFF));//Frnt
        //objs.push_back(new Sphere(objs.size(),1e5, V3(50, 1e5, 81.6),    V3(),V3(.75,.75,.75),DIFF));//Botm
        //objs.push_back(new Sphere(objs.size(),1e5, V3(50,-1e5+81.6,81.6),V3(),V3(.75,.75,.75),DIFF));//Top
        //objs.push_back(new Sphere(objs.size(),1e5, V3(50,-1e5+83.6,81.6),V3(),V3(.75,.75,.75),DIFF));//Top
        objs.push_back(new Plane(objs.size(),V3(0,1,0),81.6,V3(),V3(.75,.75,.75),DIFF));  //Top

        objs.push_back(new Sphere(objs.size(),15,V3(25,15,45),       V3(),V3(1,1,1)*.999, REFR));//Mirr
        //objs.push_back(new Sphere(objs.size(),16.5,V3(73,16.5,78),       V3(),V3(1,1,1)*.999, REFR));//Glas
        //lighter = new Sphere(objs.size(), 600, V3(50,681.6-.27,81.6),V3(1,1,1) * 12,  V3(), DIFF);//Lite
        lighter = new Sphere(objs.size(), 400, V3(50,481.6-.5,81.6),V3(1,1,1) * 12,  V3(), DIFF);//Lite
        objs.push_back(lighter);
        //readObj("Alucy_adjust.obj",V3(25,0,60),DIFF);
        loader.load("Alucy_adjust.obj",objs,V3(25,0,60),DIFF);
        */
        
 

        std::cout << "obj total: " << objs.size() << std::endl;
        std::cout << "build tree" << std::endl;
        tree = new kdTree(objs);
        std::cout << "build tree done" << std::endl;
    }

    void readObj(std::string fn, const V3& bias, Refl_t refl) {
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
                //buf.push_back(V3(x + 30,y + 30,z + 20));
                //buf.push_back(V3(x,y,z) + V3(31,0,60)); //Al
                buf.push_back(V3(x,y,z) + bias);
            } else if (s[0] == 'f') {
                //面
                int i,j,k;
                istringstream in(s);
                std::string type;
                in >> type >> i >> j >> k;
                //objs.push_back(new Triangle(objs.size(),buf[i - 1],buf[j - 1],buf[k - 1],(buf[i - 1] - buf[j - 1]) & (buf[j - 1] - buf[k - 1]), V3(),V3(.25,.25,.25), DIFF));        
                objs.push_back(new Triangle(objs.size(),buf[i - 1],buf[j - 1],buf[k - 1],(buf[i - 1] - buf[j - 1]) & (buf[j - 1] - buf[k - 1]), V3(),V3(1,1,1)*0.5, DIFF));        
            } else {
                //std::cout << s << endl;
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
        if (focal != nullptr) delete focal;
        objs.clear();
    }
};



#endif