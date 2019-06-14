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
    struct int2 {
        int x,y;
        int2(int x_ = -1,int y_ = -1):x(x_),y(y_){}
        bool operator<(const int2& a) const {
            if (x < a.x) return true;
            else if (x == a.x && y < a.y) return true;
            else return false;
        }
        bool operator==(const int2& a) const {return x == a.x && y == a.y;}
        bool operator!=(const int2& a) const {return !(*this == a);}
    };
    struct int3 {
        int x,y,z;
        int3(int x_ = -1,int y_ = -1,int z_ = -1):x(x_),y(y_),z(z_){}
        bool operator<(const int3& a) const {
            if (x < a.x) return true;
            else if (x == a.x && y < a.y) return true;
            else if (x == a.x && y == a.y && z < a.z) return true;
            else return false;
        }
        bool operator==(const int3& a) const {return x == a.x && y == a.y && z == a.z;}
        bool operator!=(const int3& a) const {return !(*this == a);}
    };


    //void connectEdge(he::Mesh& mesh, std::map<int,int3>& face_vert3_map, std::map<int2,int2> vert2_faces_map, int face_id, const int2& order) {
    void connectEdge(he::Mesh& mesh, std::vector<int3>& face_vert3_map, std::map<int2,int2>& vert2_faces_map, std::map<int2,int>& vert2_edge_map, int face_id, const int2& order) {
        if (mesh.faces[face_id]->edge != nullptr) return;

        int3 i3 = face_vert3_map[face_id];
        int z;
        if (i3.x != order.x && i3.x != order.y) z = i3.x;
        else if (i3.y != order.x && i3.y != order.y) z = i3.y;
        else z = i3.z;
        static int i = 0;
        //std::cout << "round: " << i++ << std::endl;
        assert(z != order.x && z != order.y);

        he::Edge *e0 = new he::Edge();
        he::Edge *e1 = new he::Edge();
        he::Edge *e2 = new he::Edge();

        e0->face = e1->face = e2->face = mesh.faces[face_id];

        e0->v[0] = mesh.verts[order.x];
        e0->v[1] = mesh.verts[order.y];
        e1->v[0] = mesh.verts[order.y];
        e1->v[1] = mesh.verts[z];
        e2->v[0] = mesh.verts[z];
        e2->v[1] = mesh.verts[order.x];

        if (mesh.verts[order.x]->edge == nullptr) mesh.verts[order.x]->edge = e0;
        if (mesh.verts[order.y]->edge == nullptr) mesh.verts[order.y]->edge = e1;
        if (mesh.verts[z]->edge == nullptr) mesh.verts[z]->edge = e2;

        e0->next = e1;
        e1->next = e2;
        e2->next = e0;
        e0->prev = e2;
        e2->prev = e1;
        e1->prev = e0;

        mesh.addEdge(e0);
        mesh.addEdge(e1);
        mesh.addEdge(e2);

        mesh.faces[face_id]->edge = e0;


        vert2_edge_map[int2(e0->v[0]->id,e0->v[1]->id)] = e0->id;
        vert2_edge_map[int2(e1->v[0]->id,e1->v[1]->id)] = e1->id;
        vert2_edge_map[int2(e2->v[0]->id,e2->v[1]->id)] = e2->id;

        //从e1 e2两边扩展
        //e1
        int2 new_order(z,order.y);
        int2 faces_id = vert2_faces_map[int2(min(z,order.y),max(z,order.y))];
        int new_face_id = faces_id.x == face_id? faces_id.y:faces_id.x;
        connectEdge(mesh,face_vert3_map,vert2_faces_map,vert2_edge_map,new_face_id,new_order);

        //e2
        new_order = int2(order.x,z);
        faces_id = vert2_faces_map[int2(min(z,order.x),max(z,order.x))];
        new_face_id = faces_id.x == face_id? faces_id.y:faces_id.x;
        connectEdge(mesh,face_vert3_map,vert2_faces_map,vert2_edge_map,new_face_id,new_order);
    }


    bool checkFileName(const std::string &fn) {
        std::string format = fn.substr(fn.length() - 4, 4);
        if (format == ".obj") return true;
        else {
            std::cout << "unvalid file format: " << format << std::endl;
            return false;
        }
    }
    bool loadFile(he::Mesh& mesh,const std::string& fn) {
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

        std::map<int3,int> vert3_face_map;
        std::map<int2,int2> vert2_face2_map;
        std::map<int2,int> vert2_edge_map;
        std::vector<int3> face_vert3_map;

        for (int i = 0;i < attrib.vertices.size(); i += 3) {
            he::Vert* v = new he::Vert();
            tinyobj::real_t vx = attrib.vertices[i];
            tinyobj::real_t vy = attrib.vertices[i + 1];
            tinyobj::real_t vz = attrib.vertices[i + 2];
            v->pos = V3(vx,vy,vz);
            mesh.addVert(v);
            assert(v->id == i / 3);
        }

        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[0].mesh.num_face_vertices.size(); f++) {
            int fv = shapes[0].mesh.num_face_vertices[f];
            assert(fv == 3);

            he::Face* face = new he::Face();
            mesh.addFace(face);

            tinyobj::index_t idx[3];
            for (int i = 0;i < 3; ++i) idx[i] = shapes[0].mesh.indices[index_offset + i];

            int vert_index[3];
            V3 v[3];
            for (int i = 0;i < 3; ++i) {
                vert_index[i] = idx[i].vertex_index;
                v[i] = mesh.verts[vert_index[i]]->pos;
            }

            vert3_face_map[int3(vert_index[0],vert_index[1],vert_index[2])] = face->id;
            face_vert3_map.push_back(int3(vert_index[0],vert_index[1],vert_index[2]));

            for (int i = 0;i < 3; ++i) {
                int min_index = min(vert_index[i],vert_index[(i + 1) % 3]);
                int max_index = max(vert_index[i],vert_index[(i + 1) % 3]);

                int2 i2(min_index,max_index);
                auto itr = vert2_face2_map.find(i2);
                if (itr == vert2_face2_map.end()) {
                    int2 faces_id(face->id,-1);
                    vert2_face2_map[i2] = faces_id;
                } else if (itr->second.y == -1) {
                    itr->second.y = face->id;
                    assert(itr->second.x != face->id);
                } else {
                    std::cout << "??? error" << std::endl;
                }
            }            

            V3 normal = (v[0] - v[1]) & (v[1] - v[2]);
            normal = normal.norm();
            double d = (-1) * (normal * v[0]);
            face->p = V4(normal.x,normal.y,normal.z,d);

            index_offset += fv;
        }
        int3 order = face_vert3_map[0];
        connectEdge(mesh,face_vert3_map,vert2_face2_map,vert2_edge_map,0,int2(order.x,order.y));

        //处理pair
        for (int i = 0;i < mesh.edges.size(); ++i) {
            int2 i2(mesh.edges[i]->v[1]->id,mesh.edges[i]->v[0]->id);
            mesh.edges[i]->pair = mesh.edges[vert2_edge_map[i2]];
        }
        return true;
    }


    void dumpFile(he::Mesh& mesh, std::string& fn) {
        FILE* fp = fopen(fn.c_str(),"w");
        std::map<int,int> idxMap;
        for (int i = 0, k = 0;i < mesh.verts.size(); ++i) {
            if (!mesh.vertEnable[i]) continue;
            fprintf(fp,"v %f %f %f\n",mesh.verts[i]->pos[0],mesh.verts[i]->pos[1],mesh.verts[i]->pos[2]);
            idxMap[mesh.verts[i]->id] = ++k;
        }

        for (int i = 0;i < mesh.faces.size(); ++i) {
            if (!mesh.faceEnable[i]) continue;
            he::Edge *e = mesh.faces[i]->edge;
            he::Vert *v0 = e->v[0], *v1 = e->next->v[0], *v2 = e->next->next->v[0];
            assert(v0 == e->next->next->next->v[0]);

            fprintf(fp,"f %d %d %d\n",idxMap[v0->id],idxMap[v1->id],idxMap[v2->id]);
        }

    }
};

#endif