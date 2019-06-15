#include "he_mesh.hpp"
#include "obj.hpp"
#include "timer.hpp"
#include <cstring>


#include <iostream>
int main(int argc, char** argv) {
    Object obj;
    obj.loadFile(std::string(argv[1]));
    double ratio = atof(argv[3]);
    obj.setRatio(ratio);
    obj.simplify();

    obj.dumpFile(std::string(argv[2]));
    return 0;
}