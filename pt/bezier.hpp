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
        memcpy(x,x_,sizeof(double) * num);
        memcpy(y,y_,sizeof(double) * num);


        //预处理导数
        for (int i = 0;i < num - 1; ++i) {
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
        //改为私有
        double x_buf[MAX_CONTROL];
        double y_buf[MAX_CONTROL];
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
        //改为私有
        //使用低阶de casteljau
        double x_buf[MAX_CONTROL];
        double y_buf[MAX_CONTROL];
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
    }

};

class Bezier: public Object {
public:
    Bezier2D curve;

    Bezier(int id_, double* x_, double* y_, int num_, V3 e_, V3 c_, Refl_t refl_, double ns_ = 1.5)
        : Object(id_,e_,c_,refl_,ns_),curve(x_,y_,num_) {}
    Bezier(int id_, double* x_, double* y_, int num_, V3 e_, std::string fn_, Refl_t refl_, double ns_ = 1.5)
        : Object(id_,e_,fn_,refl_,ns_),curve(x_,y_,num_) {}   
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
        

        //纹理
        
        V3 tmp = curve.pos(seed.x);
        V3 intersect_p = r.pos(seed.z);
        res.b = (tmp.y - curve.minY) / (curve.maxY - curve.minY);
        res.a = atan2(intersect_p.z,intersect_p.x);
        if (res.a < 0) res.a += 2 * PI;
        res.a /= 2 * PI;
        
        //res.a = seed.y;
        //res.b = seed.x;



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
        V3 p = curve.pos(u);
        double sin2pv = sin(2 * PI * v);
        double cos2pv = cos(2 * PI * v);
        double a = des.y * 2 * PI * cos2pv * p.x;
        double b = -2 * PI * des.x * p.x;
        double c = 2 * PI * sin2pv * p.x * des.y;
        return V3(a,b,c).norm();
    }

    V3 F(const Ray& r, const V3& seed) {
        //F(u,v,t) = S(u,v) - L(t)
        return pos(seed.x,seed.y) - r.pos(seed.z);
    }

    M3 F_jacobi(const Ray& r, const V3& seed) {
        V3 des = curve.dir(seed.x);
        V3 cp = curve.pos(seed.x);
        
        double sin2pv = sin(2 * PI * seed.y);
        double cos2pv = cos(2 * PI * seed.y);
        //列向量
        V3 c0(des.x * cos2pv, des.y, des.x * sin2pv);
        V3 c1(-2 * PI * sin2pv * cp.x, 0 , 2 * PI * cos2pv * cp.x);
        V3 c2 = r.d * (-1);
        return M3(c0,c1,c2);
    }

    M3 F_jacobi(const Ray& r, double u, double v, double t) {
        V3 des = curve.dir(u);
        V3 cp = curve.pos(u);
        
        double sin2pv = sin(2 * PI * v);
        double cos2pv = cos(2 * PI * v);
        //列向量
        V3 c0(des.x * cos2pv, des.y, des.x * sin2pv);
        V3 c1(-2 * PI * sin2pv * cp.x, 0 , 2 * PI * cos2pv * cp.x);
        V3 c2 = r.d * (-1);
        return M3(c0,c1,c2);
    }


    bool newton(const Ray& r,V3& seed) {
        //初值u,v,t  解出精确解  牛顿迭代法
        //S(u,v) - L(t) = 0  解u,v,t
        //std::cout << "------start-newton------" << std::endl;
        
        //跟aabb相交测试
        double tmin[3],tmax[3];
        std::pair<V3, V3> box = this->aabb();
        for (int i = 0;i < 3; ++i) {
            if (fabs(r.d[i]) > EPS) {
                double t1 = (box.first[i] - r.o[i]) / r.d[i];
                double t2 = (box.second[i] - r.o[i]) / r.d[i];
                tmin[i] = min(t1,t2);
                tmax[i] = max(t1,t2);
            } else {
                tmin[i] = INF;
                tmax[i] = 2 * INF;
            }
        }

        double tmin_ = max(tmin[0],tmin[1],tmin[2]);
        double tmax_ = min(tmax[0],tmax[1],tmax[2]);

        if (tmin_ >= tmax_) return false;

        double u[MAX_RAND_SEED], v[MAX_RAND_SEED], t[MAX_RAND_SEED];
        bool is_intersect[MAX_RAND_SEED] = {false};
        int valid_num = 0;
        
        for (int i = 0;i < MAX_RAND_SEED; ++i) {
            unsigned short X[3]={i + 1, i * i + 2, i * i * i + 3};
            u[i] = 2 * PI * i / MAX_RAND_SEED;
            v[i] = erand48(X);
            t[i] = (tmax_ - tmin_) * erand48(X) + tmin_;

            //牛顿迭代法
            for (int k = 0;k < MAX_NEWTON_ITER; ++k) {
                V3 f = this->pos(u[i],v[i]) - r.pos(t[i]);
                M3 jacobi = this->F_jacobi(r,u[i],v[i],t[i]);

                double det = jacobi.det();

                if (fabs(det) < EPS || fabs(det) > 1e15 || fabs(f.len()) < EPS) break;

                M3 rev = jacobi.reverse();
                
                V3 error = rev.dot(f);

                u[i] -= error[0];
                v[i] -= error[1];
                t[i] -= error[2];
            }

            double delta = (this->pos(u[i],v[i]) - r.pos(t[i])).len();
            if (fabs(delta) < EPS && u[i] >= 0 && u[i] <= 1 && t[i] > 0) {
                is_intersect[i] = true;
                valid_num++;
            } else {
                is_intersect[i] = false;
            }

        }

        //if (valid_num > 0) std::cout << "valid_num: " << valid_num << std::endl;

        //找t最小值
        double t_final = INF;
        for (int i = 0;i < MAX_RAND_SEED; ++i) {
            if (is_intersect[i] && t[i] < t_final && t[i] > EPS && u[i] >= 0 && u[i] <= 1 && v[i] >= 0 && v[i] <= 1) {
                t_final = t[i];
                seed.x = u[i];
                seed.y = v[i];
                seed.z = t[i];
            }
        }

        if (t_final < INF) return true;
        else return false;
    }


    std::pair<V3,V3> aabb() const override {
        double x = max(fabs(curve.minX),fabs(curve.maxX));
        double y = max(fabs(curve.minY),fabs(curve.minY));
        /*
        std::cout << "--bezier   aabb---" << std::endl;
        std::cout << x << ", " << y << ", " << std::endl;
        std::cout << "-----------aabb---" << std::endl;
        */

        //return std::make_pair(V3(-x,curve.minY,-x),V3(x,curve.maxY,x));
        return std::make_pair(V3(-x,-x,-x),V3(x,x,x));
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
        FILE* fp = fopen("bezier_mesh.obj","w");
        for (int i = 1;i < points.size(); ++i) {
            fprintf(fp,"v %f %f %f\n", points[i].x,points[i].y,points[i].z);
        }
        for (int i = 1;i < meshes.size(); ++i) {
            fprintf(fp,"f %d %d %d\n", meshes[i][0],meshes[i][1],meshes[i][2]);
        }
        
    }
};



#endif