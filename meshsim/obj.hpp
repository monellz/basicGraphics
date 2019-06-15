#ifndef _OBJ_
#define _OBJ_


#include "he_mesh.hpp"
#include "objProcessor.hpp"
#include "kdtree.hpp"
#include "timer.hpp"
#include "pairHeap.hpp"


class Object {
private:
    he::Mesh mesh;
    double threshold;
    double ratio;
    ObjProcessor processor;
    kdTree tree;
    Timer timer;
    PairHeap heap;


    void selectPair() {
        
        //这里考虑边
        //O(e)
        for (int i = 0;i < mesh.edges.size(); ++i) {
            if (mesh.edges[i]->v[0]->id < mesh.edges[i]->v[1]->id) {
                heap.insert(he::Pair(mesh.edges[i]));
            }
        }


        tree.build(mesh.verts.data(),mesh.verts.size());

        //O(n)
        
        int non_edge_pair_count = 0;
        for (int i = 0;i < mesh.verts.size(); ++i) {
            ValidVertPair vp(mesh.verts[i],threshold);
            //O(log n)
            tree.search(vp);
            //std::cout << "i: " << i << "  vp size: " << vp.std_pairs.size() << std::endl;

            for (int k = 0;k < vp.std_pairs.size(); ++k) {
                if (!he::is_edge(vp.std_pairs[k].first,vp.std_pairs[k].second) && vp.std_pairs[k].first->id < vp.std_pairs[k].second->id) {
                    non_edge_pair_count++;
                    heap.insert(he::Pair(vp.std_pairs[k]));
                }

            }
        }
        std::cout << "non-edge pair num: " << non_edge_pair_count << std::endl;
        
    }
public:
    Object():threshold(DEFAULT_THRESHOLD),ratio(0.5){}

    void setThreshold(double t) {
        threshold = t;
    }
    void setRatio(double r) {
        printf("set ratio: %lf\n",r);
        ratio = r;
    }

    void loadFile(std::string fn) {
        timer.start();
        bool acc = processor.loadFile(mesh,fn);
        timer.stop();
        timer.printInfo("load file: " + fn);

        if (!acc) {
            exit(1);
        }
    }

    void dumpFile(std::string fn) {
        timer.start();
        processor.dumpFile(mesh,fn);
        timer.stop();
        timer.printInfo("dump file: " + fn);
    }
    void printState() {
        printf("[current state] v: %d/%lu  e: %d/%lu  f: %d/%lu\n",mesh.vertCount,mesh.verts.size(),mesh.edgeCount,mesh.edges.size(),mesh.faceCount,mesh.faces.size());
    }

    void simplify() {
        printf("simplify start...\n");
        printState();
        timer.start();
        //初始化Q值
        timer.start();
        mesh.calculateError(); //error!!!
        timer.stop();
        timer.printInfo("calculate Q matrix");

        mesh.checkTotal();

        //选择pair
        timer.start();
        selectPair();
        timer.stop();
        timer.printInfo("select Pair");

        
        //循环简化
        //mesh.deleteEdge(mesh.edges[0]);
        //mesh.deleteEdge(mesh.edges[2]);
        timer.start();
        heap.iterate(mesh, int(mesh.faces.size() * ratio));
        timer.stop();
        timer.printInfo("heap iterate");

        //输出文件

        printf("simplify done...\n");
        timer.stop();
        timer.printInfo("total simplify");

        printState();

        timer.start();
        mesh.checkTotal();
        timer.stop();
        timer.printInfo("check manifold");
    }

};

#endif