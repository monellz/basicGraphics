#ifndef _PAIRHEAP_
#define _PAIRHEAP_

#include "he_mesh.hpp"
#include "heap.hpp"

#include <queue>
#include <set>
#include <map>
class PairHeap {
    struct cmp {
        bool operator()(he::Pair* & a, he::Pair* & b) const {
            return a->cost < b->cost;
        }
        bool operator()(he::Pair*const & a, he::Pair*const & b) const {
            return a->cost < b->cost;
        }
        
    };
    typedef std::map<he::Vert*, std::set<he::Pair*,cmp > > vert2pairMap;

    std::vector<he::Pair> pairs;
    Heap<he::Pair*, cmp> heap;
public:
    int heapSize() {
        return heap.size;
    }
    int pairSize() {
        return pairs.size();
    }

    void insert(const he::Pair& p) {
        pairs.push_back(p);
        //堆中插入等pairs数组插入结束后进行
    }

    void iterate(he::Mesh& mesh, int target_face_num) {
        std::cout << "iterate start..." << std::endl;
        std::cout << "target face num: " << target_face_num << std::endl;

        assert(heap.size == 0);
        for (int i = 0;i < pairs.size(); ++i) {
            heap.insert(&pairs[i]);
        }

        vert2pairMap vert2pair;
        for (int i = 0;i < pairs.size(); ++i) {
            vert2pair[pairs[i].v[0]].insert(&pairs[i]);
            vert2pair[pairs[i].v[1]].insert(&pairs[i]);
        }


        int round = 0;
        while (!heap.empty() && mesh.faceCount > target_face_num) {
            //printf("\rround: %d  face: %d/%lu                      ",round++, mesh.faceCount,mesh.faces.size());
            //printf("round: %d  face: %d/%lu\n",round, mesh.faceCount,mesh.faces.size());
            round++;

            //mesh.checkTotal();
            //std::cout << "round first check done" << std::endl;
            he::Pair* min_cost_pair = heap.pop();
            he::Vert* v0 = min_cost_pair->v[0];
            he::Vert* v1 = min_cost_pair->v[1];

            if (min_cost_pair->edge != nullptr) {
                //边收缩
                //assert(min_cost_pair->edge != nullptr);
                assert(min_cost_pair->edge->id >= 0 && min_cost_pair->edge->id < mesh.edgeEnable.size());
                if (mesh.edgeEnable[min_cost_pair->edge->id] == false || !mesh.vertEnable[min_cost_pair->edge->v[0]->id] || !mesh.vertEnable[min_cost_pair->edge->v[1]->id]) {
                    //std::cout << "   not delete" << std::endl;
                    continue;
                }
                assert(mesh.vertEnable[min_cost_pair->edge->v[0]->id]);
                assert(mesh.vertEnable[min_cost_pair->edge->v[1]->id]);

                //如果翻转，则不允许
                if (!min_cost_pair->checkMeshInversion()) continue;
                if (!mesh.deleteEdge(min_cost_pair->edge)) continue;           
 
            } else {
                //non_edge_part 收缩
                if (!mesh.mergeVert(v0,v1)) continue;
                //std::cout << "non-edge contraction" << std::endl;
            }

            min_cost_pair->v[0]->pos = min_cost_pair->bestPos;
            min_cost_pair->v[0]->error += min_cost_pair->v[1]->error;
            //mesh.checkTotal();
            //std::cout << "round second check done" << std::endl;
 
            std::set<he::Pair*,cmp>& v0_pairs = vert2pair[v0];
            std::set<he::Pair*,cmp>& v1_pairs = vert2pair[v1];

            //处理v0相关联的pair
            //重新计算，在堆中进行位置更新
            {
                //std::cout << "v0_pair   size: " << v0_pairs.size() << std::endl;
                for (auto itr = v0_pairs.begin(); itr != v0_pairs.end(); ++itr) {
                    he::Pair* pair = *itr;
                    if (pair == min_cost_pair) continue;
                    if (pair->edge != nullptr && mesh.edgeEnable[pair->edge->id] == false) continue;
                    
                    //pair->updateVert();
                    double prev_cost = pair->cost;
                    pair->recalculate();
                    //std::cout << "prev_cost, cost: " << prev_cost << ", " << pair->cost << std::endl;

                    //堆中更新
                    if (pair->cost < prev_cost) heap.up(pair->id);
                    else heap.down(pair->id);
                }
            }

            //处理v1相关联的pair
            {
                //std::cout << "v1_pair   size: " << v1_pairs.size() << std::endl;
                for (auto itr = v1_pairs.begin(); itr != v1_pairs.end(); ++itr) {
                    he::Pair* pair = *itr;
                    if (pair == min_cost_pair) continue;
                    if (pair->edge != nullptr && mesh.edgeEnable[pair->edge->id] == false) continue;

                    //pair->updateVert();
                    double prev_cost = pair->cost;
                    pair->recalculate();
                    //std::cout << "prev_cost, cost: " << prev_cost << ", " << pair->cost << std::endl;

                    if (pair->cost < prev_cost) heap.up(pair->id);
                    else heap.down(pair->id);

                    if (pair->edge == nullptr) {
                        he::Vert*& v = (pair->v[0] == v1)? pair->v[0]: pair->v[1];
                        assert(v == v1);
                        v = v0;
                    }
                    //把这个pair放到v0关联中去
                    v0_pairs.insert(pair);
                }
            }

            //mesh.checkTotal();
            //std::cout << "round third check done" << std::endl;
        }
        printf("\niterator done.. total round: %d\n",round);

    }
    
};
#endif