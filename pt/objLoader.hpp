#ifndef _OBJLOADER_
#define _OBJLOADER_

#include "sphere.hpp"
#include "plane.hpp"
#include "triangle.hpp"


#define TINYOBJLOADER_IMPLEMENTATION
#define TINYOBJLOADER_USE_DOUBLE
#include "tiny_obj_loader.h"
struct ObjLoader {
    bool load(std::string fn, std::vector<Object*>& objs, const V3& bias, Refl_t refl) {
        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;

        std::string warn, err;

        bool ret = tinyobj::LoadObj(&attrib,&shapes,&materials,&warn,&err,fn.c_str());
        if (!warn.empty()) std::cout << warn << std::endl;
        if (!err.empty()) std::cout << err << std::endl;
        if (!ret) return false;

        if (shapes.size() != 1) {
            std::cout << "error! not single shape" << std::endl;
            return false;
        }


        for (size_t s = 0;s < shapes.size(); s++) {
            size_t index_offset = 0;
            for (size_t f = 0;f < shapes[s].mesh.num_face_vertices.size(); f++) {
                int fv = shapes[s].mesh.num_face_vertices[f];
                V3 pts[3];
                f2 tuv[3];
                for (size_t v = 0;v < fv; v++) {
                    //do something
                    tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                    tinyobj::real_t vx = attrib.vertices[3 * idx.vertex_index + 0];
                    tinyobj::real_t vy = attrib.vertices[3 * idx.vertex_index + 1];
                    tinyobj::real_t vz = attrib.vertices[3 * idx.vertex_index + 2];

                    pts[v] = V3(vx,vy,vz) + bias;
                    //TODO:材质!
                    tinyobj::real_t red = attrib.colors[3 * idx.vertex_index + 0];
                    tinyobj::real_t green = attrib.colors[3 * idx.vertex_index + 1];
                    tinyobj::real_t blue = attrib.colors[3 * idx.vertex_index + 2];
                    //std::cout << "rgb: " << red << ", " << green << ", " << blue << std::endl;

                    tuv[v][0] = attrib.texcoords[3 * idx.texcoord_index + 0];
                    tuv[v][1] = attrib.texcoords[3 * idx.texcoord_index + 1];
                }
                index_offset += fv;

                //objs.push_back(new Triangle(objs.size(),pts[0],pts[1],pts[2],(pts[0] - pts[1]) & (pts[1] - pts[2]), V3(), V3(1,1,1) * 0.3, refl)); //ppm
                //objs.push_back(new Triangle(objs.size(),pts[0],pts[1],pts[2],(pts[0] - pts[1]) & (pts[1] - pts[2]), V3(), V3(1,1,1) * 0.5, refl)); //pt god ray
                objs.push_back(new Triangle(objs.size(),pts[0],pts[1],pts[2],(pts[0] - pts[1]) & (pts[1] - pts[2]), V3(), V3(1,1,1) * 0.999, refl));
                //objs.push_back(new Triangle(objs.size(),pts[0],pts[1],pts[2],(pts[0] - pts[1]) & (pts[1] - pts[2]), V3(), "veliero-tri.jpeg",tuv[0],tuv[1],tuv[2], refl));
            }
        }
    }

};

#endif