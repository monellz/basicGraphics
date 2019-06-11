#ifndef _HE_MESH_
#define _HE_MESH_

#include "linear.hpp"


struct he_vert {
    V3 pos;
    he_edge* edge;
    M4 error;
    int id;
    he_vert():edge(nullptr),id(-1){}

    
    bool calculateMat() {
        he_edge* e = edge;
        do {
            error += M4(e->face->p);
            e = e->pair->next;

            assert(e != nullptr);
        } while (e != edge);
    }
};

struct he_face {
    he_edge* edge; //面上的半边之一
    V4 p; //平面参数 ax + by + cz + d = 0 p = (a,b,c,d)  a^2 + b^2 + c^2 = 1
};


struct he_edge {
    he_vert* v[2]; //半边对应的顶点 v[0]->v[1]
    he_edge* pair; //对应的半边
    he_face* face; //对应的面
    he_edge* next; //在该面上的下一条半边
    he_edge* prev; //上一条半边
};

struct Pair {
    he_edge* edge;

    V3 bestPos;
    double cost;

    Pair(he_edge* e):edge(e),cost(INF) {
        calculateBestPoint();
    }
    void updateCost(double new_cost, const V3& new_pos) {
        if (new_cost < cost) {
            cost = new_cost;
            bestPos = new_pos;
        }
    }

    void calculateBestPoint() {
        he_vert *v0 = edge->v[0], *v1 = edge->v[1];
        //确保之前已经计算好了顶点的error
        M4 Q_ = v0->error + v1->error;
        
        double det = Q_.det4Affine();


        //TODO: 需要比较这些吗?
        double error_0 = Q_.quadric(v0->pos);
        updateCost(error_0,v0->pos);
        double error_1 = Q_.quadric(v1->pos);
        updateCost(error_1,v1->pos);
        double error_mid = Q_.quadric((v0->pos + v1->pos) / 2);
        updateCost(error_mid,(v1->pos + v0->pos) / 2);

        if (fabs(det) < EPS) { 
            //不可逆
            //考虑v0 + (v1 - v0) t,  t \in [0,1]线段
            double denominator = Q_.quadric(v0->pos - v1->pos);
            if (fabs(denominator) > EPS) {
                double t = Q_.quadric(v0->pos,v0->pos - v1->pos) / denominator;
                V3 pos = v0->pos + (v1->pos - v0->pos) * t;
                double error = Q_.quadric(pos);
                updateCost(error, pos);
            }
        } else {
            //可逆
            M4 rev = Q_.reverse();
            V3 pos = rev.lastColumn();
            double error = Q_.quadric(pos);
            updateCost(error,pos);
        }
        
    }

};

inline he_edge* is_edge(he_vert* v0, he_vert* v1) {
    he_edge* edge = v0->edge;
    if (!edge) return nullptr;
    do {
        if (edge->v[1] == v1) return edge;
        edge = edge->pair->next;
    } while (edge != v0->edge);
    return nullptr;
}

struct Mesh {
    std::vector<he_vert> verts;
    std::vector<he_edge> edges;
    std::vector<he_face> faces;
    
    void addVert(he_vert& v) {
        v.id = verts.size();
        verts.push_back(v);
    }
    void addEdge(const he_edge& e) {
        edges.push_back(e);
    }
    void addFace(const he_face& f) {
        faces.push_back(f);
    }

    void calculateError() {
        //计算所有顶点的误差矩阵
        for (int i = 0;i < verts.size(); ++i)
            verts[i].calculateMat();
    }
};



#endif