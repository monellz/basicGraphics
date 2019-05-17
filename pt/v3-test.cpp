#include "v3.hpp"


using namespace std;
int main() {
    V3 v(1,2,3);
    cout << v[0] << "," << v[1] << "," << v[2] << endl;

    v[0] = 0.1;
    cout << v[0] << "," << v[1] << "," << v[2] << endl;

    v.crd.sub.x = 4.5;

    cout << v[0] << "," << v[1] << "," << v[2] << endl;
    V3 b1(2,1,0),b2(5,2,0),b3(0,0,1);
    cout << det(b1,b2,b3) << endl;

    return 0;
}