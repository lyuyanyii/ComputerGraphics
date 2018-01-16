#include <cmath>
#include <iostream>
#include <algorithm>
#include "graphics.h"
#include "bitmap.h"
#include <omp.h>
using namespace std;

int main()
{
    //Ray a(V3(0, 0, 0), V3(-1, 0, 0));
    //Sphere o(V3(4, 4, 4), 4);

    /*
    V3 *col = new V3[100 * 100];

    for (int i = 0; i < 100 * 100; ++i)
    {
        col[i] = V3(1, 1, 1);
    }

    save_to_bmp("image.bmp", 100, 100, &col[0].x, 2.2);
    */
    #ifndef _OPENMP
        fprintf(stderr, "OpenMP not supported");
        printf("No");
    #endif

    #pragma omp parallel for schedule(dynamic, 1)
    for (int i = 0; i < 10; ++i)
    {
        printf("%d\n", i);
    }

    return 0;
}
//g++ % -fopenmp -o test.o -g
