#ifndef _HEAP_
#define _HEAP_

#include <cstring>
#define DEFAULT_CAPACITY 100000

//指针堆
template<class T, typename Compare>
struct Heap {
    T* elem;
    Compare cmp;
    int capacity;
    int size;
    Heap():capacity(DEFAULT_CAPACITY),size(0){
        elem = new T[capacity];
        memset(elem,0,sizeof(T) * capacity);
    }
    ~Heap() {delete [] elem;}

    void resize(int c_) {
        T* new_elem = new T[c_];
        memset(new_elem,0,sizeof(T) * c_);
        memcpy(new_elem,elem,sizeof(T) * size);

        delete [] elem;
        elem = new_elem;
        capacity = c_;
    }

    void down(int p) {
        while(true) {
            int lc = (p << 1) + 1;
            int rc = (p + 1) << 1;
            int minone = p;
            if (lc < size && cmp(elem[lc], elem[minone])) minone = lc;
            if (rc < size && cmp(elem[rc], elem[minone])) minone = rc;
            if (minone == p) break;

            T tmp = elem[p];
            elem[p] = elem[minone];
            elem[minone] = tmp;

            elem[p]->id = p;
            elem[minone]->id = minone;

            p = minone;
        }
    }
    void up(int c) {
        while (c > 0) {
            int p = (c - 1) >> 1;
            if (cmp(elem[p],elem[c])) break;

            T tmp = elem[p];
            elem[p] = elem[c];
            elem[c] = tmp;

            elem[p]->id = p;
            elem[c]->id = c;

            c = p;
        }
    }

    void check() {
        if (size == capacity) {
            resize(capacity << 1);
        }
    }


    void insert(T item) {
        check();
        elem[size++] = item;

        elem[size - 1]->id = size - 1;
        up(size - 1);
    }

    T pop() {
        T m = elem[0];
        elem[0] = elem[size - 1];
        size--;
        down(0);
        return m;
    }

    T top() {
        return elem[0];
    }

    bool empty() {
        return size == 0;
    }

};

#endif