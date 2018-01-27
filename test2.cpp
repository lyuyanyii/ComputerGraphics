#include <cstdio>
#include <iostream>
#include <algorithm>
#include "object.h"

using namespace std;

int main()
{
    /*
    poly a = poly(1, -10) * poly(1, -20) * poly(1, -30) * poly(1, -40);
    ld x = a.newton_solver(0);
    cout << x << endl;
    cout << a.eval(x) << endl;
    cout << a.eval(5.5) << endl;
    cout << a.a.size() << endl;
    for (int i = 0; i < a.a.size(); ++i)
        cout << a.a[i] << " ";
    cout << endl;
    */
    my_bezier_obj a = my_bezier_obj(V3(0, 0, 0), 1);
    Ray r(V3(0, 2, 0), unit(V3(1, 1, 1)));
    ld t = D_INF;
    V3 n;
    cout << a.intersect(r, t, n) << endl;
    cout << t << endl;
    return 0;
}
