#ifndef _KDTREE_TEMPLATE_
#define _KDTREE_TEMPLATE_
#include "v3.hpp"

template<class T>
struct Nearest {
    V3 pos; //中心
    std::vector<T*> items;
    std::vector<double> dist2s; //对应距离平方
    const double rad2; //规定最大距离平方

    double current_rad2; //目前所有xx最大距离

    Nearest(const V3& pos_, double rad_):pos(pos_),rad2(rad_ * rad_) {}
    bool insert(T* item) {
        double r2 = (item->pos - pos).len2();
        if (r2 <= rad2) {
            items.push_back(item);
            dist2s.push_back(r2);
            current_rad2 = max(r2,current_rad2);
            return true;
        } else return false;
    }
};


//kd树模板
template<class T>
class Tree {
private:
    struct Node {
        T* item = nullptr;
        Node* lc = nullptr;
        Node* rc = nullptr;
        int axis = 0; //分割轴
    };

    Node* root;
    Node* _build(T* vec, int lo,int hi,int depth) {
        //[lo, hi) size = hi - lo
        if (lo >= hi) return nullptr;
        int axis = depth % 3;
        int mid = (lo + hi) >> 1;
        
        std::nth_element(vec + lo,vec + mid,vec + hi,
            [axis](const T& lc, const T& rc) -> bool {
                return lc.pos[axis] < rc.pos[axis];
            });

        Node* node = new Node();
        node->axis = axis;
        node->item = &vec[mid];
        node->lc = _build(vec,lo,mid,depth + 1);
        node->rc = _build(vec,mid + 1,hi,depth + 1);
        return node;
    }

    void _search(Nearest<T>& np, Node* node) {
        if (node == nullptr) return;

        const int axis = node->axis;
        const double delta = np.pos[axis] - node->item->pos[axis]; //距离
        const V3 dir = node->item->pos - np.pos; //查找点指向当前光子
        const double dist2 = dir.len2(); //当前光子距查找点的距离平方
        //const double dt = // ??

        np.insert(node->item);

        if (delta > 0) {
            //在查找点左边，找光子的右孩子即可
            _search(np,node->rc);
            //搜左边
            if (delta * delta < np.rad2) _search(np,node->lc);
        } else {
            _search(np,node->lc);
            if (delta * delta < np.rad2) _search(np,node->rc);
        }
    }
public:
    Tree():root(nullptr){}
    ~Tree() {
        clear();
    }

    void build(T* vec, int size) {
        root = _build(vec,0,size,0);
    }

    void clearTree(Node* node) {
        if (node == nullptr) return;
        clearTree(node->lc);
        clearTree(node->rc);
        delete node;
    }

    void clear() {
        std::cout << "tree clear strat" << std::endl;
        clearTree(root);
        root = nullptr;
        std::cout << "tree clear done" << std::endl;
    }

    void search(Nearest<T>& np) {
        _search(np,root);
    }


};




#endif