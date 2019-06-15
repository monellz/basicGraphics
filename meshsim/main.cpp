#include "he_mesh.hpp"
#include "obj.hpp"
#include "timer.hpp"


#include <iostream>
int main(int argc, char** argv) {
    Object obj;
    obj.loadFile(std::string(argv[1]));
    //obj.loadFile("dinosaur.2k.obj");
    //obj.loadFile("sphere.obj");
    //obj.loadFile("fixed.perfect.dragon.100K.0.07.obj");
    //obj.loadFile("block.obj");
    obj.simplify();

    obj.dumpFile(std::string(argv[2]));
    //obj.dumpFile("dinosaur.2k_new.obj");
    //obj.dumpFile("sphere_new.obj");
    //obj.dumpFile("fixed.perfect.dragon.100K.0.07_new.obj");
    //obj.dumpFile("block_new.obj");

    return 0;
}