#ifndef __OBJECT_H__
#define __OBJECT_H__

#include "graphics.h"

typedef double ld;

enum MaterialType
{
    Matter = 0,
    Mirror,
    Glass,
};

//
//Sphere object
//

struct SphereObject : public Sphere
{
    V3 col;
    MaterialType type;

    SphereObject(ld r, V3, pos, V3 col, MaterialType type):
        Sphere(pos, r), col(col), type(type) {}
};

#endif
