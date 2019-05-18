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
            double split; //划分面对应的轴的位置
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

    double hit(const Ray& r) {
        //内部节点
        V3 pts;//交点

        double t;
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
    }
};

class kdTree {
private:
    int maxDepth;
    int cnt = 0;
    int* mailbox;
    int nodenum = 0;
    bool visited[500];
    int raynum = 0;
public:
    kdNode* root;
    kdTree(std::vector<Object*>& ptr) {
        std::cout << "into init" << std::endl; 
        
        //mailbox初始化
        mailbox = new int[ptr.size()];
        memset(mailbox,0,ptr.size() * sizeof(int));
        std::cout << "mailbox size: " << ptr.size() << std::endl;

        //最大深度经验公式 8 + 1.3 log(n)
        maxDepth = int(8.5 + 1.3 * log(ptr.size()));


        //计算边界盒
        std::pair<V3,V3> box = ptr[0]->aabb();
        std::vector< std::pair<V3,V3> > boxes;
        for (int i = 0;i < ptr.size(); ++i) {
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
        build(0,0,root,box, objIndex, boxes);


        std::cout << "node num: " << nodenum <<  std::endl;
        std::cout << "max depth: " << maxDepth << std::endl;
    }
    void _intersect(int rayid, const Ray& r, Intersection& res, std::vector<Object*>& objs, int axis, kdNode* cur) {
        if (cur->leaf) {
            //当前为叶子
            //逐一测试
            double t = 1e30;
            for (int i = 0;i < cur->num; ++i) {
                //检测是否重复求交
                /*
                if (visited[cur->index[i]]) {
                    //std::cout << "really visited cur->index[i]:" << cur->index[i] << std::endl;
                    if (mailbox[cur->index[i]] != rayid) {
                        std::cout << "error!: cur ray id:" << rayid << ", mailbox id:" << mailbox[cur->index[i]] << std::endl;
                    }
                    continue;
                }
                */
                if (mailbox[cur->index[i]] == rayid) {
                    //std::cout << "repeat!!" << std::endl;
                    if (mailbox[cur->index[i]] != rayid) {
                        std::cout << "error!: cur ray id:" << rayid << ", mailbox id:" << mailbox[cur->index[i]] << std::endl;
                    }
                    continue;
                }
 
                Intersection foo;
                if (objs[cur->index[i]]->intersect(r,foo) && foo.t < t) {
                    res = foo;
                    t = foo.t;
                }

                mailbox[cur->index[i]] = rayid;
                visited[cur->index[i]] = true;
                cnt++;
            }
            return;
        }        

        //当前为内部节点
        if (cur->lc->hit(r)) _intersect(rayid,r,res,objs,(axis + 1) % 3, cur->lc);
        if (cur->rc->hit(r)) _intersect(rayid,r,res,objs,(axis + 1) % 3, cur->rc);
    }

    bool intersect(const Ray& r, Intersection& res, std::vector<Object*>& objs) {
        static int id = 1;
        cnt = 0;
        memset(visited,0,sizeof(visited));
        memset(mailbox,0,sizeof(int) * objs.size());
        _intersect(id++, r,res,objs,0,root);
        //std::cout << "raynum: " << raynum << std::endl;
        //printf("id : %d\n",id);
        std::cout << cnt << std::endl;
        if (res.t > 0) return true;
    }


    void build(int axis, int depth, kdNode* cur, std::pair<V3,V3> box, std::vector<int>& objIndex, std::vector< std::pair<V3, V3> >& boxes) {
        //std::cout << "axis:" << axis << "," << "depth: " << depth << "size:" << objIndex.size() <<  std::endl;
        if (objIndex.size() < 5 || depth > maxDepth) {
            //终止 当前节点设置为叶子
            cur->leaf = true;
            cur->box = box;
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
        
        //对所有obj进行轴排序 //递增
        std::sort(objIndex.begin(),objIndex.end(),[&](int i,int j) {
            return boxes[i].second[axis] > boxes[i].second[axis];
        });

        //std::cout << "sort done" << std::endl;
        //找到中位数
        int mid = objIndex.size() >> 1;
        //std::cout << "mid: " << mid << std::endl;
        cur->split = boxes[objIndex[mid]].second[axis];
        //std::cout << "split: " << cur->split << std::endl;


        std::vector<int> left(mid + 1), right(objIndex.size() - mid - 1);
        std::copy(objIndex.begin(),objIndex.begin() + mid + 1, left.begin());
        std::copy(objIndex.begin() + mid + 1,objIndex.end(), right.begin());

        //std::cout << "left size:" << left.size() << ", " << "right size:" << right.size() << std::endl;
        cur->lc = new kdNode;
        cur->rc = new kdNode;
        nodenum += 2;

        //计算包围盒
        std::pair<V3,V3> tmp = boxes[left[0]];
        for (int i = 0;i < left.size(); ++i) {
            tmp.first = min(tmp.first,boxes[left[i]].first);
            tmp.second = max(tmp.second,boxes[left[i]].second);
        }
        build((axis + 1) % 3, depth + 1, cur-> lc, tmp,  left, boxes);

        tmp = boxes[right[0]];
        for (int i = 0;i < right.size(); ++i) {
            tmp.first = min(tmp.first,boxes[right[i]].second);
            tmp.second = max(tmp.second,boxes[right[i]].second);
        }
        build((axis + 1) % 3, depth + 1, cur-> rc, tmp, right, boxes);

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