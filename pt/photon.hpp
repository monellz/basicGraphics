#ifndef _PHOTON_
#define _PHOTON_
#include "v3.hpp"
#include "kdtree-tem.hpp"

using namespace std;
struct Photon {
    V3 pos;
    V3 dir;
    V3 power; //能量
    Photon(){}
    Photon(const V3& pos_, const V3& dir_, const V3& power_):pos(pos_),dir(dir_),power(power_){}

    double operator[] (int axis) {
        return pos[axis];
    }
};

struct NearestPhotons {
    V3 pos; //以pos为中心
    std::vector<Photon*> photons;
    std::vector<double> dist2s; //对应的距离平方
    const double rad2; //距离平方

    double current_rad2; //目前所有光子最大距离
    NearestPhotons(const V3& pos_,double rad_):pos(pos_),rad2(rad_ * rad_),current_rad2(-1){}
    bool insert(Photon* photon) {
        double r2 = (photon->pos - pos).len2();
        if (r2 <= rad2) {
            photons.push_back(photon);
            dist2s.push_back(r2);
            current_rad2 = max(r2,current_rad2);
            return true;
        } else return false;
    }
};
/*
class PhotonMap {
private:
    int max_photons,stored_photons;
    Photon* photons; //所有光子

    //kd树节点
    struct Node {
        Photon* photon = nullptr;
        Node* lc = nullptr;
        Node* rc = nullptr;
        int axis = 0; //分割轴
    };

    Node* root; //kd树root
    Node* _build(int lo, int hi,int depth) {
        //[lo, hi) size = hi - lo
        if (lo >= hi) return nullptr;
        int axis = depth % 3;
        int mid = (lo + hi) >> 1;
        
        std::nth_element(photons + lo,photons + mid,photons + hi,
            [axis](const Photon& lc, const Photon& rc) -> bool {
                return lc.pos[axis] < rc.pos[axis];
            });
        Node* node = new Node();
        node->axis = axis;
        node->photon = &photons[mid];
        node->lc = _build(lo,mid,depth + 1);
        node->rc = _build(mid + 1,hi,depth + 1);
        return node;
    }

    void _searchPhotons(NearestPhotons& np, Node* node) {
        if (node == nullptr) return;

        const int axis = node->axis;
        const double delta = np.pos[axis] - node->photon->pos[axis]; //距离
        const V3 dir = node->photon->pos - np.pos; //查找点指向当前光子
        const double dist2 = dir.len2(); //当前光子距查找点的距离平方
        //const double dt = // ??

        np.insert(node->photon);

        if (delta > 0) {
            //光子在查找点左边，找光子的右孩子即可
            _searchPhotons(np,node->rc);
            //搜左边
            if (delta * delta < np.rad2) _searchPhotons(np,node->lc);
        } else {
            _searchPhotons(np,node->lc);
            if (delta * delta < np.rad2) _searchPhotons(np,node->rc);
        }
    }

public:

    PhotonMap(int max_photons_):max_photons(max_photons_),root(nullptr){
        photons = new Photon [max_photons];
        stored_photons = 0;
    }
    ~PhotonMap() {
        clearTree(root);
        delete [] photons;
    }
    bool store(const Photon& photon) {
        if (stored_photons >= max_photons) return false;
        photons[stored_photons++] = photon;
    }
    int getStoreNum() const {
        return stored_photons;
    }
    void build() {
        root = _build(0,stored_photons,0);
    }

    void clearTree(Node* node) {
        if (node == nullptr) return;
        clearTree(node->lc);
        clearTree(node->rc);
        delete node;
    }

    void searchPhotons(NearestPhotons& np) {
        _searchPhotons(np,root);
    }

    V3 irradEstimate(const V3& pos, const V3& normal,const V3& color, double max_dist, int max_photons) {
        NearestPhotons np(pos,max_dist);
        
        this->searchPhotons(np);
        V3 flux;
        if (np.current_rad2 <= 0) {
            //cout << "no photons" << endl;
            return flux;
        } else {
            //cout << "phontons num: " << np.photons.size() << endl;
        }
        const double current_max_dist = sqrt(np.current_rad2);
        const double k = 1.1;
        for (int i = 0;i < np.photons.size(); ++i) {
            double w = 1.0 - (sqrt(np.dist2s[i]) / (k * current_max_dist));
            V3 v = (color * np.photons[i]->power) / PI;
            flux += v * w;
        }

        flux /= (1.0 - 2.0 / (3.0 * k));
        flux /= (PI * np.current_rad2);
        //std::cout << "flux : " << std::endl;
        //flux.print();
        return flux;
    }

};
*/

class PhotonMap {
private:
    int max_photons,stored_photons;
    Photon* photons; //所有光子
    Tree<Photon> tree;
public:
    PhotonMap(int max_photons_):max_photons(max_photons_){
        photons = new Photon [max_photons];
        stored_photons = 0;
    };
    ~PhotonMap() {
        delete [] photons;
    }
    bool store(const Photon& photon) {
        if (stored_photons >= max_photons) return false;
        photons[stored_photons++] = photon;
    }
    int getStoreNum() const {
        return stored_photons;
    }
 
    void build() {
        tree.build(photons,stored_photons);
    }

    void clear() {
        //清空树
        std::cout << "pm clear" << std::endl;
        tree.clear();
        //清空photon map
        stored_photons = 0;
        std::cout << "pm clear done" << std::endl;
    }

    void searchPhotons(Nearest<Photon>& np) {
        tree.search(np);
    }

    V3 irradEstimate(const V3& pos,const V3& color, double max_dist, int& photonNum) {
        Nearest<Photon> np(pos,max_dist);
        
        this->searchPhotons(np);
        V3 flux;
        if (np.current_rad2 <= 0) {
            //cout << "no photons" << endl;
            photonNum = 0;
            return flux;
        } else {
            //cout << "phontons num: " << np.photons.size() << endl;
            photonNum = np.items.size();
        }
        const double current_max_dist = sqrt(np.current_rad2);
        const double k = 1.1;
        for (int i = 0;i < np.items.size(); ++i) {
            //code filter
            //double w = 1.0 - (sqrt(np.dist2s[i]) / (k * current_max_dist));
            //V3 v = (color * np.items[i]->power) / PI;
            //flux += v * w;

            //no filter
            //flux += color * np.items[i]->power;

            //gaussian filter
            double w = 0.918 * (1 - (1 - exp(-1.953 * np.dist2s[i] / (2 * np.current_rad2))) / (1 - exp(-1.953)));
            flux += color * np.items[i]->power * w;
        }

        flux /= (1.0 - 2.0 / (3.0 * k));
        flux /= (PI * np.current_rad2);
        //std::cout << "flux : " << std::endl;
        //flux.print();
        return flux;
    }


};
#endif