#ifndef _KDTREE_
#define _KDTREE_
#include "he_mesh.hpp"
#include <algorithm>

struct ValidVertPair {
    he::Vert* vert;
    std::vector<he::stdPair> std_pairs;
    const double threshold2;

    ValidVertPair(he::Vert* vert_, double t_):vert(vert_),threshold2(t_ * t_){}

    bool insert(he::Vert* v) {
        double r2 = (v->pos - vert->pos).len2();
        if (r2 < threshold2 && vert->id < v->id) {
            std_pairs.push_back(std::make_pair(vert,v));
            return true;
        } return false;
    }
};

class kdTree {
private:
    struct Node {
        he::Vert* v;
        Node* lc;
        Node* rc;
        int axis; //分割轴
        Node():v(nullptr),lc(nullptr),rc(nullptr),axis(0){}
    };

    Node* root;

    Node* _build(he::Vert* verts, int lo, int hi, int depth) {
        if (lo >= hi) return nullptr;
        int axis = depth % 3;
        int mid = (lo + hi) >> 1;

        std::nth_element(verts + lo, verts + mid, verts + hi,
        [axis](const he::Vert& lc, const he::Vert& rc) -> bool {
            return lc.pos[axis] < rc.pos[axis];
        });                                                                                                                                     

        Node* node = new Node();
        node->axis = axis;
        node->v = &verts[mid];
        node->lc = _build(verts,lo,mid,depth + 1);
        node->rc = _build(verts,mid + 1, hi, depth + 1);
        return node;
    }

    void _search(ValidVertPair& vp, Node* node) {
        if (node == nullptr) return;

        const int axis = node->axis;
        const double delta = vp.vert->pos[axis] - node->v->pos[axis];
        const V3 dir = node->v->pos - vp.vert->pos;
        const double d2 = dir.len2();

        vp.insert(node->v);

        if (delta > 0) {
            _search(vp,node->rc);
            if (delta * delta < vp.threshold2) _search(vp,node->lc);
        } else {
            _search(vp,node->lc);
            if (delta * delta < vp.threshold2) _search(vp,node->rc);
        }
    }

public:
    kdTree():root(nullptr){}
    ~kdTree() {clear();}

    void clear() {
        clearTree(root);
        root = nullptr;
    }

    void clearTree(Node* node) {
        if (node == nullptr) return;
        clearTree(node->lc);
        clearTree(node->rc);
        delete node;
    }


    void build(he::Vert* verts, int size) {
        root = _build(verts,0,size,0);
    }

    void search(ValidVertPair& vp) {
        _search(vp,root);
    }

};



#endif