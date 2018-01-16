#ifndef __HITPOINT_H__
#define __HITPOINT_H__

typedef double ld;

struct HitPoint
{
    V3 pos, n, flux, f;
    ld r2;
    int N, idx;
};

#endif
