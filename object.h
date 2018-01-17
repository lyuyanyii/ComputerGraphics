#ifndef __OBJECT_H__
#define __OBJECT_H__

#include "graphics.h"
#include <vector>
#include <cstring>
#include <fstream>
using namespace std;

typedef double ld;

enum MaterialType
{
    Matte = 0,
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

    SphereObject(ld r, V3 pos, V3 col, MaterialType type):
        Sphere(pos, r), col(col), type(type) {}
};

struct Triangle
{
    V3  a, b, c, na, nb, nc, u, v, n, mid;
    ld uu, vv, uv, D;

    Triangle() {}
    Triangle(const V3 &a, const V3 &b, const V3 &c, const V3 &na, const V3 &nb, const V3 &nc): a(a), b(b), c(c), na(na), nb(nb), nc(nc) {
        mid = (a + b + c) / 3;
        u = b - a, v = c - a;
        n = unit( det( u, v ) );
        uu = dot( u, u );
        uv = dot( u, v );
        vv = dot( v, v );
        D = uv * uv - uu * vv;
    }

    V3 calc_norm(const V3 &p) const {
        V3 x = a - p, y = b - p, z = c - p;
        ld aa = det(y, z).norm(), bb = det(z, x).norm(), cc = det(x, y).norm();
        return (na * aa + nb * bb + nc * cc) / (aa + bb + cc);
    }

    V3 getmid() const {
        return mid;
    }

    bool intersect(const Ray &r, ld &t) const {
        V3 w0 = r.pos - a;
        ld aa = -dot( n, w0 ), 
           bb =  dot( n, r.dir );
        if (fabs(bb) < D_EPS)
            return 0;
        t = aa / bb;
        if (t < D_EPS)
            return 0;
        V3 p = r.pos + r.dir * t,
           w = p - a;

        ld wu = dot( w, u ),
           wv = dot( w, v ),
           f  = (uv * wv - vv * wu) / D,
           g  = (uv * wu - uu * wv) / D;
        
        if (f < 0 || f > 1)
            return 0;
        if (g < 0 || (f + g) > 1)
            return 0;
        return 1;
    }
};

struct KDNode
{
    KDNode *lc, *rc;
    BBox bbox;
    Triangle *tri;
    KDNode() {
        lc = rc = 0;
        tri = 0;
        bbox.reset();
    }
};

struct KDTree
{
    vector<Triangle*> tris;
    KDNode *root;

    KDTree() {}
    KDTree(vector<Triangle> &meshes) {
        /*
        for (auto it = meshes.begin(); it != meshes.end(); ++it)
            tris.push_back(it);
        */
        for (int i = 0; i < meshes.size(); ++i)
            tris.push_back(&meshes[i]);
        buildKDTree();
    }

    void buildKDTree() {
        root = split(tris, 0);
    }

    KDNode* split(const vector<Triangle*> &tris, int dep) {
        if (tris.size() == 0)
            return 0;

        //printf("%d\n", tris.size());
        KDNode *u = new KDNode();
        u->bbox.reset();
        if (tris.size() == 1)
        {
            u->tri = tris[0];
            u->bbox.update(tris[0]->a);
            u->bbox.update(tris[0]->b);
            u->bbox.update(tris[0]->c);
            return u;
        }
        V3 mid = 0;
        for (int i = 0; i < tris.size(); ++i)
        {
            mid += tris[i]->getmid();
            u->bbox.update(tris[i]->a);
            u->bbox.update(tris[i]->b);
            u->bbox.update(tris[i]->c);
        }
        mid = mid / (ld)tris.size();

        vector<Triangle*> left, right;
        for (int i = 0; i < tris.size(); ++i)
        {
            bool flag = 0;
            if (dep == 0)
                flag = ( tris[i]->getmid().x < mid.x );
            if (dep == 1)
                flag = ( tris[i]->getmid().y < mid.y );
            if (dep == 2)
                flag = ( tris[i]->getmid().z < mid.z );
            if (flag)
                left.push_back(tris[i]);
            else
                right.push_back(tris[i]);
        }
        u->lc = split(left,  (dep + 1) % 3);
        u->rc = split(right, (dep + 1) % 3);

        return u;
    }

    bool hit(KDNode* u, const Ray &r, ld &t, Triangle* &obj) const {
        ld t1 = -1, t2 = -1;
        if ( !u->bbox.intersect(r, t1, t2) )
            return 0;
        /*
        printf("BBox:\n");
        u->bbox.mini.print(); cout << endl;
        u->bbox.maxi.print(); cout << endl;
        cout << t1 << " " << t2 << endl;
        (r.pos + r.dir * t1).print(); cout << endl;
        */
        if ( u->tri != 0 )
        {
            ld t3 = 0;
            /*
            printf("Triangle:\n");
            u->tri->a.print(), cout << endl;
            u->tri->b.print(), cout << endl;
            u->tri->c.print(), cout << endl;
            */
            if ( !u->tri->intersect(r, t3) )
                return 0;
            if ( t3 < t )
                obj = u->tri, t = t3;
            return 1;
        }
        ld tl1, tl2, tr1, tr2;
        bool flagl = u->lc && u->lc->bbox.intersect( r, tl1, tl2 ),
             flagr = u->rc && u->rc->bbox.intersect( r, tr1, tr2 );
        bool ans = 0;
        if ( tl1 < tr1 || !flagr )
        {
            if (flagl)
                ans |= hit( u->lc, r, t, obj );
            if (flagr && ( obj == 0 || t > tr1 ))
                ans |= hit( u->rc, r, t, obj );
        }
        else
        {
            if (flagr)
                ans |= hit( u->rc, r, t, obj );
            if (flagl && ( obj == 0 || t > tl1 ))
                ans |= hit( u->lc, r, t, obj );
        }
        return ans;
    }
};
//
// locate at (60, 0, 110), scale : 0.5
//
struct MeshObject
{
    vector<Triangle> meshes;
    BBox bbox;
    KDTree tree;
    V3 col;
    MaterialType type;

    MeshObject(const string& file_name, const V3 &col, MaterialType type): col(col), type(type)
    {
        vector<V3> pts;
        vector<V3> norms;

        ifstream fin(file_name.c_str());
        //freopen(file_name, "r", stdin);

        bbox.reset();
        string str;
        while (getline(fin, str))
        {
            if (str[0] == 'v')
            {
                if (str[1] != 'n')
                {
                    ld x, y, z;
                    sscanf(str.c_str(), "v%lf%lf%lf", &x, &y, &z);
                    V3 p = V3(x, y, z);
                    bbox.update(p);
                    pts.push_back(p);
                }
                else
                {
                    ld x, y, z;
                    sscanf(str.c_str(), "vn%lf%lf%lf", &x, &y, &z);
                    V3 p = V3(x, y, z);
                    norms.push_back(unit(p));
                }
            }
            if (str[0] == 'f')
            {
                int a, b, c, na, nb, nc, tmp;
                sscanf(str.c_str(), "f %d/%d/%d %d/%d/%d %d/%d/%d", &a, &tmp, &na, &b, &tmp, &nb, &c, &tmp, &nc);
                Triangle mesh = Triangle(pts[a - 1], pts[b - 1], pts[c - 1], norms[na - 1], norms[nb - 1], norms[nc - 1]);

                meshes.push_back(mesh);
            }
        }

    }

    void Trans(const V3 &v, ld scale) {
        for (auto it = meshes.begin(); it != meshes.end(); ++it)
        {
            it->a = it->a * scale + v;
            it->b = it->b * scale + v;
            it->c = it->c * scale + v;
        }
    }

    void Build() {
        tree = KDTree(meshes);
        tree.buildKDTree();
    }

    bool intersect(const Ray &r, ld &t, V3 &n) const {
        Triangle *obj = 0;
        t = D_INF;
        bool ans = tree.hit(tree.root, r, t, obj);
        if (!ans)
            return 0;
        V3 x = r.pos + r.dir * t;
        n = obj->calc_norm(x);
        return 1;
    }
};


#endif
