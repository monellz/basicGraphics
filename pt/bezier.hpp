#ifndef _BEZIER_
#define _BEZIER_

#include "obj.hpp"
#include "utils.hpp"
struct Bezier2D {
    //z轴为0的2d bezier曲线
    //控制点的x.y坐标
    double* x;
    double* y;
    double* dx; //导数控制点 n(P_{i + 1} - P_{i}) 
    double* dy;
    //控制点个数
    int num; 
    //阶数为num - 1

    double maxX,maxY,minX,minY;

    double* x_buf;
    double* y_buf;
    Bezier2D(double* x_, double* y_, int n):num(n){
        x = new double[num];
        y = new double[num];
        dx = new double[num - 1]; //导数曲线少一个控制点
        dy = new double[num - 1];
        x_buf = new double[num];
        y_buf = new double[num];
        memcpy(x,x_,sizeof(double) * num);
        memcpy(y,y_,sizeof(double) * num);


        //预处理导数
        for (int i = 0;i < num - 2; ++i) {
            dx[i] = (num - 1) * (x[i + 1] - x[i]);
            dy[i] = (num - 1) * (y[i + 1] - y[i]);
        }

        //计算包围盒的x,y
        maxX = minX = x[0];
        maxY = minY = y[0];
        for (int i = 1;i < num; ++i) {
            maxX = max(maxX, x[i]);
            minX = min(minX, x[i]);
            maxY = max(maxY, y[i]);
            minY = min(minY, y[i]);
        }
        
    }

    V3 pos(double t) {
        memcpy(x_buf, x, sizeof(double) * num);
        memcpy(y_buf, y, sizeof(double) * num);
        for (int k = 1; k < num; ++k) {
            for (int p = k; p < num; ++p) {
                x_buf[p] = x_buf[p] * t + x_buf[p - 1] * (1 - t);
                y_buf[p] = y_buf[p] * t + y_buf[p - 1] * (1 - t);
            }
        }
        return V3(x_buf[num - 1], y_buf[num - 1]);
    }

    //切向量
    V3 dir(double t) {
        //使用低阶de casteljau
        memcpy(x_buf, dx, sizeof(double) * (num - 1));
        memcpy(y_buf, dy, sizeof(double) * (num - 1));
        for (int k = 1; k < num - 1; ++k) {
            for (int p = k; p < num - 1; ++p) {
                x_buf[p] = x_buf[p] * t + x_buf[p - 1] * (1 - t);
                y_buf[p] = y_buf[p] * t + y_buf[p - 1] * (1 - t);
            }
        }
        return V3(x_buf[num - 2], y_buf[num - 2]);
    }

    ~Bezier2D() {
        delete [] x;
        delete [] y;
        delete [] dx;
        delete [] dy;
        delete [] x_buf;
        delete [] y_buf;
    }

};

class Bezier: public Object {
public:
    Bezier2D curve;

    Bezier(int id_, double* x_, double* y_, int num_, V3 e_, V3 c_, Refl_t refl_, double ns_ = 1.5)
        : Object(id_,e_,c_,refl_,ns_),curve(x_,y_,num_) {}
    
    bool intersect(const Ray& r, Intersection& res) override {
        V3 seed;
        //seed = (u,v,t)
        bool converge = newton(r,seed);
        if (!converge) return false;

        res.t = seed.z;
        res.id = id;

        //求法向量
        res.n = this->normal(seed.x,seed.y);
        //进入/出去
        if (res.n.dot(r.d) < 0) {
            res.into = true;
        } else {
            res.into = false;
            res.n = -res.n;
        }
        
        //纹理??
        return true;
    }

    V3 pos(double u, double v) {
        //绕y轴旋转 u,v in [0,1]
        //点为(x(u) cos 2pi v , y(u) , x(u) sin 2 pi v)
        V3 p = curve.pos(u);
        double theta = v * 2 * PI;
        return V3(p.x * cos(theta), p.y, p.x * sin(theta));
    }

    V3 normal(double u,double v) {
        V3 des = curve.dir(u);
        V3 p = this->pos(u,v);
        double sin2pv = sin(2 * PI * v);
        double cos2pv = cos(2 * PI * v);
        double a = des.y * 2 * PI * cos2pv * p.z;
        double b = (-2 * PI) * (sin2pv * sin2pv * des.z * p.x + cos2pv * cos2pv * des.x * p.z);
        double c = 2 * PI * sin2pv * des.y;
        return V3(a,b,c).norm();
    }

    bool newton(const Ray& r,V3& seed) {
        //初值u,v,t  解出精确解  牛顿迭代法
        //S(u,v) - L(t) = 0  解u,v,t
        V3 last;
        for (int i = 0;i < 6; ++i) {
            last = seed;
            V3 des = curve.dir(seed.x);
            V3 cp = curve.pos(seed.x);
            V3 c0(cos(2 * PI * seed.y) * des.x, des.y, sin(2 * PI * seed.y) * des.z);
            V3 c1(-2 * PI * sin(2 * PI *seed.y) * cp.x, 0 ,2 * PI * cos(2 * PI * seed.y) * cp.z);
            V3 c2 = r.d * (-1);

            M3 m(c0,c1,c2);
            
            seed = seed - m.reverse().dot(pos(seed.x,seed.y) - r.pos(seed.z));
        }
                

        //判断迭代结果是否收敛
        double delta = (last - seed).len2();
        std::cout << "tolerance: " << delta << std::endl;
        if (delta > EPS) return false;
        if (seed.x < 0 || seed.x > 1 || seed.y < 0 || seed.y > 1 || seed.z < 0 || seed.z > 1) return false;
        return true;
    }


    std::pair<V3,V3> aabb() const override {
        double x = max(fabs(curve.minX),fabs(curve.maxX));
        double y = max(fabs(curve.minY),fabs(curve.minY));

        return std::make_pair(V3(-x,-y,-x),V3(x,y,x));
    }

    void mesh() {
        std::vector<V3> points;
        std::vector<int4> meshes;
        points.push_back(V3());
        meshes.push_back(int4());

        int nu = 10;
        int nv = 1000;
        for (int i = 0;i <= nu; ++i) {
            fprintf(stderr,"\rmeshing %5.2f%%",100 * (double)i / nu);
            for (int j = 0;j <= nv; ++j) {
                double u = (double)i / nu;
                double v = (double)j / nv;
                points.push_back(this->pos(u,v));
                if (i != 0 && j != 0) {
                    int p1 = (i - 1) * (nv + 1) + j;
                    int p2 = p1 + 1;
                    int p3 = i * (nv + 1) + j;
                    int p4 = p3 + 1;
                    meshes.push_back(int4(p1,p2,p3));
                    meshes.push_back(int4(p2,p3,p4));
                }
            }
        }

        //写文件
        FILE* fp = fopen("besizer_mesh.obj","w");
        for (int i = 1;i < points.size(); ++i) {
            fprintf(fp,"v %f %f %f\n", points[i].x,points[i].y,points[i].z);
        }
        for (int i = 1;i < meshes.size(); ++i) {
            fprintf(fp,"f %d %d %d\n", meshes[i][0],meshes[i][1],meshes[i][2]);
        }
        
    }
};



#endif