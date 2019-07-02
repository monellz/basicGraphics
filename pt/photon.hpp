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