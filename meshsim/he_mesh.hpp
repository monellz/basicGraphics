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

    Edge(const Edge& p) {
        id = p.id;
        v[0] = p.v[0];
        v[1] = p.v[1];
        pair = p.pair;
        face = p.face;
        next = p.next;
        prev = p.prev;
    }
};

inline Edge* is_edge(Vert* v0, Vert* v1) {
    //找到一条v0指向v1的边
    Edge* edge = v0->edge;
    if (!edge) return nullptr;
    int count = 0;
    do {
        count++;
        if (count > LOOP_INFINIT) {
            std::cout << "warning! is_edge infinit loop" << std::endl;
            return nullptr;
        }
        if (edge->v[1] == v1) return edge;
        edge = edge->pair->next;
    } while (edge != v0->edge);
    return nullptr;
}


typedef std::pair<Vert*, Vert*> stdPair;

struct Pair {
    Edge* edge; //若edge == nullptr　代表两个点之间之前未链接
    Vert* v[2];

    V3 bestPos;
    double cost;

    int id;

    Pair(Edge* e):edge(e),cost(INF),id(-1) {
        v[0] = e->v[0];
        v[1] = e->v[1];
        calculateBestPoint();
    }

    Pair(Vert* v0,Vert* v1):edge(nullptr),cost(INF),id(-1) {
        v[0] = v0;
        v[1] = v1;
        calculateBestPoint();
    }

    Pair(const Pair& p) {
        edge = p.edge;
        v[0] = p.v[0];
        v[1] = p.v[1];
        bestPos = p.bestPos;
        cost = p.cost;
        id = p.id;
    }

    Pair(const stdPair& pair):edge(nullptr),cost(INF) {
        v[0] = pair.first;
        v[1] = pair.second;
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

    void updateVert() {
        assert(edge != nullptr);
        v[0] = edge->v[0];
        v[1] = edge->v[1];
    }

    void recalculate() {
        cost = INF;
        calculateBestPoint();
    }
    void calculateBestPoint() {
        if (edge != nullptr) updateVert();
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

    bool checkMeshInversion() {
        //检查这个pair是否会导致附近的face法向量翻转
        //只考虑边收缩
        assert(edge != nullptr);
        updateVert();
        
        //遍历v0 v1的邻接面除待删的边
        Edge* v0_v1_edge = is_edge(v[0],v[1]);
        assert(v0_v1_edge != nullptr);
        for (int i = 0;i < 2; ++i) {
            Edge* e = v[i]->edge;
            do {
                Face* face = e->face;
                
                if (face != v0_v1_edge->face && face != v0_v1_edge->pair->face) {
                    V3 face_v[3];
                    //计算face3个点中非v[i]的两个点
                    {
                        int k = 0;
                        Edge* face_e = face->edge;
                        do {
                            if (face_e->v[0] != v[i]) {
                                face_v[k++] = face_e->v[0]->pos;
                            } else {
                                face_v[k++] = bestPos;
                            }
                            face_e = face_e->next;
                        } while (face_e != face->edge);
                    }

                    //计算新法向量
                    V3 new_normal = ((face_v[0] - face_v[1]) & (face_v[1] - face_v[2])).norm();
                    //如果翻转　则直接不允许这个pair收缩
                    if (new_normal * V3(face->p.x,face->p.y,face->p.z) <= 0) return false;
                }
                e = e->pair->next;
            } while (e != v[i]->edge);
        }
        return true;
    }

};

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

    bool deleteEdge(Edge* edge) {
        assert(edgeEnable[edge->id]);
        //删除一条存在的边 v0->v1
        //把edge->v[1]删除　留下edge->v[0]
        //因此实际上就是删掉v[1]并重新链接
        //将会删除两个三角面 6条半边(包括当前边) 1个顶点
        /*

        正常情况
            v_up
          /      \
        v0 --e--> v1
           <----
          \      /
            v_down 
        */

        Vert* v1 = edge->v[1];
        Vert* v0 = edge->v[0];
 
        //所有非v0，　且与v0无边的v1的邻居
        //除此之外如果正常情况应该还有3个点(v0和两个与v0有边的顶点)
        std::vector<Vert*> neighbor_vert;
        std::vector<Edge*> neighbor_edge;

        int judge_count = 0; //对非v0,且与v0有边的v1的邻居进行计数
        {
            Edge* e = edge->pair;
            assert(e->v[0] == v1);
            assert(e->v[1] == v0);

            do {
                assert(e->v[0] == v1);
                if (e->v[1] != v0 && !is_edge(e->v[1],v0)) {
                    neighbor_vert.push_back(e->v[1]);
                    neighbor_edge.push_back(e);
                    //std::cout << "neighbor v,e: " << e->v[1]->id << ", " << e->id << std::endl;
                }
                //delete!!
                if (e->v[1] != v0 && is_edge(e->v[1],v0)) judge_count++;

                e = e->pair->next;
            } while (e != edge->pair);
        }

        //std::cout << "vert neighbor: " << neighbor_vert.size() << std::endl;
        //std::cout << "edge neighbor: " << neighbor_edge.size() << std::endl;

        //如果非v0, 且与v0有边的v1的邻居个数不等于2(>2)则，可能会将立体变成平面，直接跳过，不删除
        if (judge_count != 2) return false;


        //删除节点v[1]
        vertEnable[v1->id] = false;
        vertCount--;
        //删除边
        edgeEnable[edge->id] = false;
        edgeEnable[edge->pair->id] = false;
        //std::cout << "delete edge id: " << edge->id << std::endl;
        //std::cout << "delete edge id: " << edge->pair->id << std::endl;
        edgeCount -= 2;
 


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

        /*
        if (neighbor_vert.size() == 1) {
            //assert(is_edge(v_up, neighbor_vert[0]));
            std::cout << " v_up, v_down, v0, v1: " << v_up->id << ", " << v_down->id << ", " << v0->id << ", " << v1->id << std::endl;
            Edge* e = neighbor_edge[0];
            assert(e->v[0] == v1);
            do {
                if (e != edge && e != edge->pair) assert(edgeEnable[e->id]);
                assert(e->v[0] == v1);
                std::cout << "v1 neibor edge: " << e->id << std::endl;
                std::cout << "  to vert: " << e->v[1]->id << std::endl;

                e = e->pair->next;
            } while (e != neighbor_edge[0]);
        }
        */

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
            //面的边改变
            e_down->face->edge = e_down;
            assert(edgeEnable[e_down->face->edge->id]);
            assert(e_down->v[0] == v_down && e_down->v[1] == v0);
            assert(e_down->next->v[0] == v1 && e_down->next->v[1] == e_next->v[1]);
            assert(e_down->next->next->v[0] == e_next->v[1] && e_down->next->next->v[1] == v_down);

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
            //面的边改变
            e_up->face->edge = e_up;
            assert(edgeEnable[e_up->face->edge->id]);

            //检查三角关系
            //std::cout << "e_up check "<< e_up->id << ", " << e_next->id << ", " << e_next_next->id << std::endl;
            checkEdge(e_up);
            //std::cout << "e_up check done" << std::endl;
        }

        //更新其余的顶点邻居        
        {
            for (int i = 0;i < neighbor_vert.size(); ++i) {
                assert(neighbor_edge[i]->v[1] == neighbor_vert[i]);

                /*
                if (neighbor_edge[i]->v[0] != v1) {
                    std::cout << "neighbor e v[0]: " << neighbor_edge[i]->v[0] << ", " << neighbor_edge[i]->v[0]->id << std::endl;
                    std::cout << "v1: " << v1 << ",  " << v1->id << std::endl;
                    std::cout << "neighbor e v[0] enable: " << vertEnable[neighbor_edge[i]->v[0]->id] << std::endl;
                    std::cout << "v1 enable: " << vertEnable[v1->id] << std::endl;


                    for (int k = 0;k < neighbor_vert.size(); ++k) {
                        std::cout << neighbor_vert[k]->id << std::endl;
                    }
                }
                */
                assert(neighbor_edge[i]->v[0] == v1);
                assert(neighbor_edge[i]->pair->v[1] == v1);
                assert(neighbor_edge[i]->pair->v[0] == neighbor_vert[i]);

                //将边的起点更改为v0  对应边的终点改为v0
                neighbor_edge[i]->v[0] = v0;
                neighbor_edge[i]->pair->v[1] = v0;
            }
        }

        return true;
    }

    bool mergeVert(Vert* v0, Vert* v1) {
        //合并两个节点 这两个节点之间不存在边!
        Edge* edge = is_edge(v0,v1);
        if (edge != nullptr || !vertEnable[v0->id] || !vertEnable[v1->id]) return false;
        
        //把v[1]合并到v[0]
        //遍历v[1]的边即可
        Edge* e = v1->edge;
        std::cout << "check" << std::endl;
        checkEdge(e);
        std::cout << "check right" << std::endl;
        int i = 0;
        do {
            std::cout << "i: " << i++ << std::endl;
            assert(e->v[0] == v1);
            assert(e->pair->v[1] == v1);

            e->v[0] = v0;
            e->pair->v[1] = v0;

            e = e->pair->next;
        } while (e != v1->edge);

        vertEnable[v1->id] = false;
        vertCount--;

        return true;
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

        //检查顶点enable
        assert(vertEnable[e0->v[1]->id]);
        assert(vertEnable[e1->v[1]->id]);
        assert(vertEnable[e2->v[1]->id]);

        assert(v == v->edge->next->next->next->v[0]);

        Edge* e = v->edge;
        do {
            assert(e != nullptr);
            checkEdge(e);

            /*
            if (e->v[0] != v) {
                std::cout << "error e->v[0]: " << e->v[0] << ", " << e->v[0]->id << std::endl;
                std::cout << "      e->v[1]: " << e->v[1] << ", " << e->v[1]->id << std::endl;
                std::cout << "      e: " << e << ", " << e->id << std::endl;
                std::cout << "      v: " << v << ", " << v->id << std::endl;
            }
            */
            assert(e->v[0] == v);
            assert(e->next->next->next->v[0] == v);
            assert(faceEnable[e->face->id]);
            assert(edgeEnable[e->face->edge->id]);
            assert(e->id >= 0 && e->id < edges.size());

            e = e->pair->next;
        } while (e != v->edge);
    }

    void checkEdge(Edge* e) {
        Edge *e0 = e, *e1 = e0->next, *e2 = e1->next;
        assert(e0 != nullptr);
        assert(e1 != nullptr);
        assert(e2 != nullptr);
        assert(edgeEnable[e0->id]);
        assert(edgeEnable[e1->id]);
        assert(edgeEnable[e2->id]);
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