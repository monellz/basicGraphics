#ifndef _OBJPROCESSOR_
#define _OBJPROCESSOR_

#include <string>
#include <iostream>
#include <cstdio>


#include "he_mesh.hpp"


//第三方库
#define TINYOBJLOADER_IMPLEMENTATION
#define TINYOBJLOADER_USE_DOUBLE
#include "tiny_obj_loader.h"

struct ObjProcessor {
    bool checkFileName(const std::string &fn) {
        std::string format = fn.substr(fn.length() - 4, 4);
        if (format == "obj") return true;
        else {
            std::cout << "unvalid file format: " << format << std::endl;
            return false;
        }
    }
    bool loadFile(Mesh& mesh,const std::string& fn) {
        if (!checkFileName(fn)) return false;

        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;

        std::string warn;
        std::string err;

        bool ret = tinyobj::LoadObj(&attrib,&shapes,&materials,&warn,&err,fn.c_str());

        if (!warn.empty()) {
            std::cout << warn << std::endl;
        }

        if (!err.empty()) {
            std::cout << err << std::endl;
        }

        if (!ret) {
            std::cout << "reject!" << std::endl;
            return false;
        }
           
        if (shapes.size() != 1) {
            std::cout << "error!  multiple or zero shapes" << std::endl;
            return false;
        }

        for (size_t s = 0; s < shapes.size(); s++) {
            size_t index_offset = 0;
            for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
                int fv = shapes[s].mesh.num_face_vertices[f];

                for (size_t v = 0; v < fv; v++) {
                    //do something
                    //TODO
                }
                index_offset += fv;
            }
        }
         
    }


    bool dumpFile() {

    }
};

#endif