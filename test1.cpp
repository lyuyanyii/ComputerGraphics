#include <cmath>
#include <iostream>
#include <algorithm>
#include "graphics.h"
#include "bitmap.h"
#include "object.h"
using namespace std;

int main()
{
    /*
    MeshObject a = MeshObject("3d-model.obj");

    cout << a.pts[0].x << " " << a.pts[0].y << " " << a.pts[0].z << endl;
    cout << a.meshes[0].a->x << " " << a.meshes[0].b->x << " " << a.meshes[0].c->x << endl;
    a.bbox.maxi.print();
    cout << endl;
    a.bbox.mini.print();
    cout << endl;

    Ray r(0, V3(1, 0, 0));
    Triangle mesh(V3(1, 0, 0), V3(2, 1, -1), V3(3, -1, -1), 0, 0, 0);
    ld t = 0;
    cout << mesh.intersect(r, t) << endl;
    */
    MeshObject a = MeshObject("1.obj", 0.99, MaterialType::Glass);
    a.Build();
    Ray r(V3(0, 0.5, 0), V3(1e-4, 1, 1e-4));
    ld t = 0;
    V3 n = 0;
    cout << a.intersect(r, t, n) << endl;
    cout << t << endl;
    n.print();

    return 0;
}
//g++ % -fopenmp -o test.o -g
