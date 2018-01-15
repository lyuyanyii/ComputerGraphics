#ifndef __GRAPHICS_H__
#define __GRAPHICS_H__

#include <cmath>

typedef double ld;

static const ld D_PI = 3.1415926535897932384626433832795;
static const ld D_INF = 1e20;
static const ld D_EPS = 1e-4;

#define sqr(_) ((_)*(_))

//
//3-d Vector
//

struct V3
{
    ld x, y, z;

    V3(): x(0), y(0), z(0) {}
    V3(ld k): x(k), y(k), z(k) {}
    V3(ld x, ld y, ld z): x(x), y(y), z(z) {}

    inline V3 operator +(const V3 &a) const {
        return V3(x + a.x, y + a.y, z + a.z);
    }
    inline V3 operator -(const V3 &a) const {
        return V3(x - a.x, y - a.y, z - a.z);
    }
    inline V3 operator *(const V3 &a) const {
        return V3(x * a.x, y * a.y, z * a.z);
    }
    inline V3 operator /(const V3 &a) const {
        return V3(x / a.x, y / a.y, z / a.z);
    }
    inline V3 operator - () const {
        return V3(-x, -y, -z);
    }
    inline V3& operator += (const V3 &a) {
        x += a.x, y += a.y, z += a.z;
        return (*this);
    }
    inline V3& operator -= (const V3 &a) {
        x -= a.x, y -= a.y, z -= a.z;
        return (*this);
    }
    inline V3& operator *= (const V3 &a) {
        x *= a.x, y *= a.y, z *= a.z;
        return (*this);
    }
};

inline V3 unit(const V3 &v) 
{
    ld len = sqrt(sqr(v.x) + sqr(v.y) + sqr(v.z));
    return v / len;
}
inline ld dot(const V3 &a, const V3 &b) 
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline ld det(const V3 &a, const V3 &b)
{
    return V3(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x );
}

//
//Ray
//

struct Ray
{
    V3 pos, dir;

    Ray(): pos(), dir() {}
    Ray(const V3 &pos, const V3 &dir): pos(pos), dir(dir) {}
};

struct Sphere
{
    V3 pos;
    ld r;

    Sphere(): pos(), r() {}
    Sphere(const V3 &pos, ld r): pos(pos), r(r) {}

    inline ld intersect(const Ray &ray) const 
    {
        V3 v = pos - ray.pos;
        ld a = dot(v, ray.dir);
        ld delta = sqr(a) + sqr(r) - dot(v, v);

        if (delta < 0)
            return D_INF;
        
        delta = sqrt(delta);
        if (a - delta > D_EPS)
            return a - delta;
        if (a + delta > D_EPS)
            return a + delta;
        return D_INF;
    }
};


//
//Bounding Box
//

struct BBox
{
    V3 mini, maxi;

    inline void update(const V3 &v) 
    {
        mini.x = min(mini.x, v.x);
        mini.y = min(mini.y, v.y);
        mini.z = min(mini.z, v.z);

        maxi.x = max(maxi.x, v.x);
        maxi.y = max(maxi.y, v.y);
        maxi.z = max(maxi.z, v.z);
    }

    inline void reset() 
    {
        mini = V3( D_INF,  D_INF,  D_INF);
        maxi = V3(-D_INF, -D_INF, -D_INF);
    }
};

#endif