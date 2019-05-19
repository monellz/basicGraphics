#ifndef _KDTREE_
#define _KDTREE_
#include "utils.hpp"
#include "obj.hpp"
#include <vector>
#include <algorithm>
#include "ray.hpp"

struct kdNode {
    bool leaf = false; //是否为叶子节点
    std::pair<V3, V3> box; //包围盒
    union {
        //内部节点
        struct {
            kdNode* lc; //左孩子
            kdNode* rc; //右孩子
            int innernum;
        };
        //叶子节点
        struct {
            int num; //叶子下的obj个数
            int* index;
        };
    };

    bool hit(const Ray& r) {
        //slab求交
        //内部节点
        /*
        double tmin[3],tmax[3];
        for (int i = 0;i < 3; ++i) {
            if (r.d[i] != 0) {
                tmin[i] = (box.first[i] - r.o[i]) / r.d[i];
                tmax[i] = (box.second[i] - r.o[i]) / r.d[i];
            } else {
                tmin[i] = 1e30;
                tmax[i] = 1e30;
            }
        }

        double tmin_ = max(tmin[0],tmin[1],tmin[2]);
        double tmax_ = min(tmax[0],tmax[1],tmax[2]);
        if (tmin_ < tmax_) return true;
        else return false;
        */
        
        for (int i = 0;i < 3; ++i) {
            double t1 = -1, t2 = -1;
            if (r.d[i] != 0) {
                t1 = (box.first[i] - r.o[i]) / r.d[i];
                t2 = (box.second[i] - r.o[i]) / r.d[i];
            }
            if (t1 >= EPS) {
                V3 pts = r.pos(t1);
                bool ok = true;
                for (int j = 0;j < 3; ++j) {
                    if (j == i) continue;
                    if (pts[j] < box.first[j] - EPS || pts[j] > box.second[j] + EPS) {
                        ok = false;
                        break;
                    }
                }
                if (ok) return true;
            }
            if (t2 >= EPS) {
                V3 pts = r.pos(t2);
                bool ok = true;
                for (int j = 0;j < 3; ++j) {
                    if (j == i) continue;
                    if (pts[j] < box.first[j] + EPS || pts[j] > box.second[j] - EPS) {
                        ok = false;
                        break;
                    }
                }
                if (ok) return true;
            }
        }
        return false;
        
        /*
        if (r.d.x != 0) {
            if (r.d.x > 0) t = (box.first.x - r.o.x) / r.d.x;
            else t = (box.second.x - r.o.x) / r.d.x;
            if (t > 0) {
                pts = r.pos(t);
                if (box.first.y < pts.y && pts.y < box.second.y && box.first.z < pts.z && pts.z < box.second.z)
                    return t;
            }
        }
        if (r.d.y != 0) {
            if (r.d.y > 0) t = (box.first.y - r.o.y) / r.d.y;
            else t = (box.second.y - r.o.y) / r.d.y;
            if (t > 0) {
                pts = r.pos(t);
                if (box.first.z < pts.z && pts.z < box.second.z && box.first.x < pts.x && pts.x < box.second.x)
                    return t;
            }
        }
        if (r.d.z != 0) {
            if (r.d.z > 0) t = (box.first.z - r.o.z) / r.d.z;
            else t = (box.second.z - r.o.z) / r.d.z;
            if (t > 0) {
                pts = r.pos(t);
                if (box.first.x < pts.x && pts.x < box.second.x && box.first.y < pts.y && pts.y < box.second.y)
                    return t;
            }
        }
        return -1;
        */
    }
};

class kdTree {
private:
    int maxDepth;
    int cnt = 0;
    int nodenum = 0;
    int raynum = 0;
    int* mailbox;
public:
    kdNode* root;
    kdTree(std::vector<Object*>& ptr) {
        std::cout << "into init" << std::endl; 
        

        //最大深度经验公式 8 + 1.3 log(n)
        maxDepth = int(8.5 + 1.3 * log(ptr.size()));
        mailbox = new int[ptr.size()];
        memset(mailbox,0,sizeof(int) * ptr.size());


        //计算边界盒
        std::pair<V3,V3> box = ptr[0]->aabb();
        std::vector< std::pair<V3,V3> > boxes;
        boxes.push_back(box);
        for (int i = 1;i < ptr.size(); ++i) {
            std::pair<V3, V3> aabb = ptr[i]->aabb();
            box.first = min(box.first,aabb.first);
            box.second = max(box.second,aabb.second);
            boxes.push_back(aabb); //下标与ptr对应
        } 

        //建树
        root = new kdNode;
        nodenum++;
        std::vector<int> objIndex;
        objIndex.resize(ptr.size());
        for (int i = 0; i < ptr.size(); ++i) objIndex[i] = i;
        std::cout << "init done" << std::endl;
        build(0,root,box, objIndex, boxes);


        std::cout << "node num: " << nodenum <<  std::endl;
        std::cout << "max depth: " << maxDepth << std::endl;
    }
    void _intersect(const Ray& r, Intersection& res, std::vector<Object*>& objs,kdNode* cur) {
        if (cur->leaf) {
            //当前为叶子
            //逐一测试
            double t = 1e30;
            for (int i = 0;i < cur->num; ++i) {
                Intersection foo;
                if (objs[cur->index[i]]->intersect(r,foo)) {
                    if (res.t <= 0) res = foo;
                    else if (res.t > foo.t) res = foo;
                    cnt++;
                }
            }
            return;
        }        

        //当前为内部节点
        if (cur->lc->hit(r)) _intersect(r,res,objs,cur->lc);
        //_intersect(r,res,objs,cur->lc);
        if (cur->rc->hit(r)) _intersect(r,res,objs, cur->rc);
        //_intersect(r,res,objs, cur->rc);
    }

    bool intersect(const Ray& r, Intersection& res, std::vector<Object*>& objs) {
        //cnt = 0;
        //bool visited[objs.size()];
        //memset(visited,0,sizeof(bool) * objs.size());
        _intersect(r,res,objs,root);
        //std::cout << "raynum: " << raynum << std::endl;
        //printf("id : %d\n",id);
        //if (cnt < objs.size() && cnt > 0) std::cout << cnt << std::endl;
        if (res.t > 0) return true;
    }


    void build(int depth, kdNode* cur, std::pair<V3,V3> box, std::vector<int>& objIndex, std::vector< std::pair<V3, V3> >& boxes) {
        //std::cout << "axis:" << axis << "," << "depth: " << depth << "size:" << objIndex.size() <<  std::endl;
        if (objIndex.size() <= 3 || depth > maxDepth) {
            //终止 当前节点设置为叶子
            cur->leaf = true;
            cur->box = box;
            cout << "leaf box:" << endl;
            cur->box.first.print();
            cur->box.second.print();
            cout << "leaf box end---" << endl;
            cur->index = new int[objIndex.size()];
            for (int i = 0;i < objIndex.size(); ++i) cur->index[i] = objIndex[i];
            cur->num = objIndex.size();
            //std::cout << "depth = " << depth << std::endl;
            return;
        }

        //否则为内部节点
        cur->leaf = false;
        cur->innernum = objIndex.size();
        cur->box = box;
        cout << "inner box:" << endl;
        cur->box.first.print();
        cur->box.second.print();
        cout << "inner box end---" << endl;
        
        int axis = (box.second - box.first).maxAxis(); //选择最长的轴

        //对所有obj进行轴排序 //递增
        std::sort(objIndex.begin(),objIndex.end(),[&](int a,int b) {
            return boxes[a].second[axis] < boxes[b].second[axis];
        });

        //std::cout << "sort done" << std::endl;
        //找到中位数
        int mid = objIndex.size() >> 1;
        //std::cout << "mid: " << mid << std::endl;


        std::vector<int> left(objIndex.begin(),objIndex.begin() + mid);
        std::vector<int> right(objIndex.begin() + mid, objIndex.end());

        std::cout << "axis:" << axis  << ", left size:" << left.size() << ", " << "right size:" << right.size() << std::endl;
        std::cout << "left" << endl;
        for (int i = 0;i < left.size(); ++i) cout << left[i] << "," << boxes[left[i]].second[axis] << "  ";
        cout << endl << "right" << endl;
        for (int i = 0;i < right.size(); ++i) cout << right[i] << "," << boxes[right[i]].second[axis] << "  ";
        cout << endl;
        cur->lc = new kdNode;
        cur->rc = new kdNode;
        nodenum += 2;

        //计算包围盒
        std::pair<V3,V3> tmp = boxes[left[0]];
        for (int i = 1;i < left.size(); ++i) {
            tmp.first = min(tmp.first,boxes[left[i]].first);
            tmp.second = max(tmp.second,boxes[left[i]].second);
        }
        build( depth + 1, cur-> lc, tmp,  left, boxes);

        tmp = boxes[right[0]];
        for (int i = 1;i < right.size(); ++i) {
            tmp.first = min(tmp.first,boxes[right[i]].first);
            tmp.second = max(tmp.second,boxes[right[i]].second);
        }
        build(depth + 1, cur-> rc, tmp, right, boxes);

    }
    
    void travel(kdNode* cur) {
        if (cur->leaf) {
            std::cout << "leaf!! num: " << cur->num  << std::endl;
            for (int i = 0;i < cur->num; ++i) {
                std::cout << cur->index[i] << std::endl;
            }
            std::cout << "left done---" << std::endl;
            return;
        }

        std::cout << "inner!!   num: " << cur->innernum << std::endl;
        travel(cur->rc);
        travel(cur->lc);
    }
};




#endif