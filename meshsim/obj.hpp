#ifndef _OBJ_
#define _OBJ_


#include "he_mesh.hpp"
#include "objProcessor.hpp"


class Object {
private:
    Mesh mesh;
    double threshold;
    double ratio;
    ObjProcessor processor;
public:
    Object():threshold(0.5),ratio(0.5){}

    void setThreshold(double t) {
        threshold = t;
    }
    void setRatio(double r) {
        ratio = r;
    }

    void loadFile(std::string fn) {
        bool acc = processor.loadFile(mesh,fn);
        if (!acc) {
            exit(1);
        }
    }

    void simplify() {
        //初始化Q值
        mesh.calculateError();

        //选择pair

        
        //循环简化


        //输出文件
    }

};

#endif