#ifndef _HE_MESH_
#define _HE_MESH_

#include "linear.hpp"
namespace he {

struct Edge;

struct Vert {
    V3 pos;
    Edge* edge; //这条边以当前v为起点(v[0])
    M4 error;
    int id;
    Vert():edge(nullptr),id(-1){}
};

struct Face {
    Edge* edge; //面上的半边之一
    V4 p; //平面参数 ax + by + cz + d = 0 p = (a,b,c,d)  a^2 + b^2 + c^2 = 1
    int id;
    Face():edge(nullptr),id(-1){}
};


struct Edge {
    int id;
    Vert* v[2]; //半边对应的顶点 v[0]->v[1]
    Edge* pair; //对应的半边
    Face* face; //对应的面
    Edge* next; //在该面上的下一条半边
    Edge* prev; //上一条半边
    Edge():pair(nullptr),face(nullptr),next(nullptr),prev(nullptr),id(-1) {
        v[0] = v[1] = nullptr;
    }
};

typedef std::pair<Vert*, Vert*> stdPair;

struct Pair {
    Edge* edge; //若edge == nullptr　代表两个点之间之前未链接
    Vert* v[2];

    V3 bestPos;
    double cost;

    Pair(Edge* e):edge(e),cost(INF) {
        v[0] = e->v[0];
        v[1] = e->v[1];
        calculateBestPoint();
    }

    Pair(Vert* v0,Vert* v1):edge(nullptr),cost(INF) {
        v[0] = v0;
        v[1] = v1;
        calculateBestPoint();
    }

    Pair(const Pair& p) {
        edge = p.edge;
        bestPos = p.bestPos;
        cost = p.cost;
    }

    void printInfo() const {
        std::cout << "edge: " << edge << std::endl;
        std::cout << "v[0]xyz: " << v[0]->pos[0] << ", " << v[0]->pos[1] << ", " << v[0]->pos[2] << std::endl;
        std::cout << "v[1]xyz: " << v[1]->pos[0] << ", " << v[1]->pos[1] << ", " << v[1]->pos[2] << std::endl;
        std::cout << "cost: " << cost << std::endl;
    }

    void updateCost(double new_cost, const V3& new_pos) {
        if (new_cost < cost) {
            cost = new_cost;
            bestPos = new_pos;
        }
    }

    void recalculate() {
        cost = INF;
        calculateBestPoint();
    }
    void calculateBestPoint() {
        //确保之前已经计算好了顶点的error
        M4 Q_ = v[0]->error + v[1]->error;
        
        double det = Q_.det4Affine();


        //TODO: 需要比较这些吗?
        double error_0 = Q_.quadric(v[0]->pos);
        updateCost(error_0,v[0]->pos);
        double error_1 = Q_.quadric(v[1]->pos);
        updateCost(error_1,v[1]->pos);
        double error_mid = Q_.quadric((v[0]->pos + v[1]->pos) / 2);
        updateCost(error_mid,(v[1]->pos + v[0]->pos) / 2);

        if (fabs(det) < EPS) { 
            //不可逆
            //考虑v0 + (v1 - v0) t,  t \in [0,1]线段
            double denominator = Q_.quadric(v[0]->pos - v[1]->pos);
            if (fabs(denominator) > EPS) {
                double t = Q_.quadric(v[0]->pos,v[0]->pos - v[1]->pos) / denominator;
                V3 pos = v[0]->pos + (v[1]->pos - v[0]->pos) * t;
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

    void update2opti() {
        //把v0更新到最优点
        v[0]->pos = bestPos;
        v[0]->error += v[1]->error;
        //外面进行边删除
    }

};

inline Edge* is_edge(Vert* v0, Vert* v1) {
    //找到一条v0指向v1的边
    Edge* edge = v0->edge;
    if (!edge) return nullptr;
    do {
        if (edge->v[1] == v1) return edge;
        edge = edge->pair->next;
    } while (edge != v0->edge);
    return nullptr;
}

inline void calculateMat4Vert(Vert* v) {
    Edge* e = v->edge;
    do {
        v->error += M4(e->face->p);
        e = e->pair->next;

        assert(e != nullptr);
    } while (e != v->edge);
}

struct Mesh {
    std::vector<Vert*> verts;
    std::vector<Edge*> edges;
    //面都是三角片!!
    std::vector<Face*> faces;

    std::vector<bool> vertEnable;
    std::vector<bool> edgeEnable;
    std::vector<bool> faceEnable;

    int vertCount = 0;
    int edgeCount = 0;
    int faceCount = 0;
    
    void addVert(Vert* v) {
        v->id = verts.size();
        verts.push_back(v);
        vertEnable.push_back(true);
        vertCount++;
    }
    void addEdge(Edge* e) {
        e->id = edges.size();
        edges.push_back(e);
        edgeEnable.push_back(true);
        edgeCount++;
    }
    void addFace(Face* f) {
        f->id = faces.size();
        faces.push_back(f);
        faceEnable.push_back(true);
        faceCount++;
    }

    void calculateError() {
        //计算所有顶点的误差矩阵
        for (int i = 0;i < verts.size(); ++i) {
            calculateMat4Vert(verts[i]);
            //std::cout << "---" << std::endl;
            //verts[i]->error.print();
        }
    }

    void deleteEdge(Edge* edge) {
        assert(edgeEnable[edge->id]);
        //删除一条存在的边 v0->v1
        //把edge->v[1]删除　留下edge->v[0]
        //因此实际上就是删掉v[1]并重新链接
        //将会删除两个三角面 6条半边(包括当前边) 1个顶点

        Vert* v1 = edge->v[1];
        Vert* v0 = edge->v[0];
        //删除节点v[1]
        vertEnable[v1->id] = false;
        vertCount--;
        //删除边
        edgeEnable[edge->id] = false;
        edgeEnable[edge->pair->id] = false;
        //std::cout << "delete edge id: " << edge->id << std::endl;
        //std::cout << "delete edge id: " << edge->pair->id << std::endl;
        edgeCount -= 2;
        /*
            v_up
          /      \
        v0 --e--> v1
           <----
          \      /
            v_down 
        */
        //所有非v0，　且与v0无边的邻居
        //除此之外还有3个点(v0和两个与v0有边的顶点)
        std::vector<Vert*> neighbor_vert;
        std::vector<Edge*> neighbor_edge;

        {
            Edge* e = edge->pair;
            assert(e->v[0] == v1);
            assert(e->v[1] == v0);

            do {
                if (e->v[1] != v0 && !is_edge(e->v[1],v0)) {
                    neighbor_vert.push_back(e->v[1]);
                    neighbor_edge.push_back(e);
                }

                e = e->pair->next;
            } while (e != edge->pair);
        }

        //std::cout << "vert neighbor: " << neighbor_vert.size() << std::endl;
        //std::cout << "edge neighbor: " << neighbor_edge.size() << std::endl;


        //检查与v0有边的两个顶点v_up,v_down
        Vert* v_down = edge->next->v[1];
        //e_down: v_down -> v0
        Edge* e_down = edge->next->next; 
        Vert* v_up = edge->pair->next->v[1];
        //e_up: v0 -> v_up
        Edge* e_up = edge->pair->next;
        assert(e_down->v[0] == v_down && e_down->v[1] == v0);
        assert(e_up->v[0] == v0 && e_up->v[1] == v_up);
        //三角关系
        assert(e_up->next->next->v[1] == v0);
        assert(e_down->next->next->v[1] == v_down);

        //更改v0 v_up v_down的edge　避免被删
        v_up->edge = e_up->pair;
        assert(v_up->edge->v[0] == v_up && v_up->edge->v[1] == v0);
        v_down->edge = e_down;
        assert(v_down->edge->v[0] == v_down && v_down->edge->v[1] == v0);        
        v0->edge = v_down->edge->pair;
        assert(v0->edge->v[0] == v0);

        //删除v1->v_down和v_down->v1这两条边
        {
            assert(e_down->prev->v[0] == v1 && e_down->prev->v[1] == v_down);
            edgeEnable[e_down->prev->id] = false;
            edgeEnable[e_down->prev->pair->id] = false;
            edgeCount -= 2;
            //std::cout << "delete edge id: " << e_down->prev->id << std::endl;
            //std::cout << "delete edge id: " << e_down->prev->pair->id << std::endl;

            faceEnable[e_down->face->id] = false;
            faceCount--;

            //重新连接形成三角片
            Edge* e_next = e_down->prev->pair->next;
            Edge* e_next_next = e_down->prev->pair->next->next;
            
            e_down->next = e_next;
            e_next->prev = e_down;
            e_down->prev = e_next_next;
            e_next_next->next = e_down;
            
            //面改变
            e_down->face = e_next->face;

            //检查三角关系
            //std::cout << "e_down check "<< e_down->id << ", " << e_next->id << ", " << e_next_next->id << std::endl;
            checkEdge(e_down);
            //std::cout << "e_down check done" << std::endl;

        }

        //删除v1->v_up和v_up->v1这两条边
        {
            assert(e_up->next->v[1] == v1 && e_up->prev->v[1] == v0);
            edgeEnable[e_up->next->id] = false;
            edgeEnable[e_up->next->pair->id] = false;
            edgeCount -= 2;
            //std::cout << "delete edge id: " << e_up->next->id << std::endl;
            //std::cout << "delete edge id: " << e_up->next->pair->id << std::endl;
            faceEnable[e_up->face->id] = false;
            faceCount--;

            //重新连接三角片
            Edge* e_next = e_up->next->pair->next;
            Edge* e_next_next = e_up->next->pair->prev;
            assert(e_next->next == e_next_next);

            e_up->next = e_next;
            e_next->prev = e_up;
            e_up->prev = e_next_next;
            e_next_next->next = e_up;

            //面改变
            e_up->face = e_next->face;

            //检查三角关系
            //std::cout << "e_up check "<< e_up->id << ", " << e_next->id << ", " << e_next_next->id << std::endl;
            checkEdge(e_up);
            //std::cout << "e_up check done" << std::endl;
        }

        //更新其余的顶点邻居        
        {
            for (int i = 0;i < neighbor_vert.size(); ++i) {
                assert(neighbor_edge[i]->v[1] == neighbor_vert[i]);
                assert(neighbor_edge[i]->v[0] == v1);
                assert(neighbor_edge[i]->pair->v[1] == v1);
                assert(neighbor_edge[i]->pair->v[0] == neighbor_vert[i]);

                //将边的起点更改为v0  对应边的终点改为v0
                neighbor_edge[i]->v[0] = v0;
                neighbor_edge[i]->pair->v[1] = v0;
            }
        }
    }

    void mergeVert(Vert* v0, Vert* v1) {
        //合并两个节点 这两个节点之间不存在边!
    }


    void checkTotal() {
        //检查所有节点结构
        for (int i = 0;i < verts.size(); ++i) {
            if (vertEnable[i]) {
                checkVert(verts[i]);
            }
        }
    }

    void checkVert(Vert* v) {
        //检查这个顶点对应的边结构
        Edge* e0 = v->edge, *e1 = e0->next, *e2 = e1->next;
        
        //检查顶点
        assert(e0->v[1] == e1->v[0]);
        assert(e1->v[1] == e2->v[0]);
        assert(e2->v[1] == e0->v[0]);

        Edge* e = v->edge;
        do {
            assert(e != nullptr);
            checkEdge(e);
            e = e->pair->next;
        } while (e != v->edge);
    }

    void checkEdge(Edge* e) {
        Edge *e0 = e, *e1 = e0->next, *e2 = e1->next;
        assert(e0 != nullptr);
        assert(e1 != nullptr);
        assert(e2 != nullptr);
        //检查三角结构
        assert(e1->prev == e0);
        assert(e2->prev == e1);
        assert(e0->prev == e2);
        assert(e0 == e2->next);

        //检查面id
        assert(e0->face->id == e1->face->id && e0->face->id == e2->face->id);
        assert(e0->face->id != -1);
    }
};


}; //namespace he


#endif