#ifndef __OBJECT_H__
#define __OBJECT_H__

#include "graphics.h"
#include <vector>
#include <cstring>
#include <fstream>
#include <assert.h>
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
        if (fabs(bb) < 0)
            return 0;
        t = aa / bb;
        if (t < 0)
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
        //buildKDTree();
    }

    void buildKDTree() {
        root = split(tris, 0);
    }

    KDNode* split(const vector<Triangle*> &tris, int dep) {
        if (tris.size() == 0)
            return 0;

        /*
        cout << tris.size() << endl;
        for (int i = 0; i < tris.size(); ++i)
        {
            cout << "Triangle:";
            tris[i]->a.print(); cout << endl;
            tris[i]->b.print(); cout << endl;
            tris[i]->c.print(); cout << endl;
        }
        */
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
        //cout << left.size() << " " << right.size() << endl;
        u->lc = split(left,  (dep + 1) % 3);
        u->rc = split(right, (dep + 1) % 3);

        return u;
    }

    bool brutehit(KDNode *u, const Ray &r, ld &t, Triangle* &obj) const
    {
        if (!u)
            return 0;
        if (u->tri != 0)
        {
            ld t1 = 0;
            if (!u->tri->intersect( r, t1 ))
                return 0;
            if (t1 < t)
                obj = u->tri, t = t1;
            return 1;
        }
        return brutehit(u->lc, r, t, obj) | brutehit(u->rc, r, t, obj);
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
        //vector<V3> norms;
        vector< vector<int> > tri_ids;

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
                    swap(x, y);
                    y = -y;
                    V3 p = V3(x, y, z);
                    bbox.update(p);
                    pts.push_back(p);
                }
                else
                {
                    /* Do nothing here since we don't use the norm provided by .obj file*/
                    ld x, y, z;
                    sscanf(str.c_str(), "vn%lf%lf%lf", &x, &y, &z);
                    swap(x, y);
                    y = -y;
                    V3 p = V3(x, y, z);
                    //norms.push_back(unit(p));
                }
            }
            if (str[0] == 'f')
            {
                int a, b, c, na, nb, nc, tmp;
                sscanf(str.c_str(), "f %d/%d/%d %d/%d/%d %d/%d/%d", &a, &tmp, &na, &b, &tmp, &nb, &c, &tmp, &nc);
                //Triangle mesh = Triangle(pts[a - 1], pts[b - 1], pts[c - 1], norms[na - 1], norms[nb - 1], norms[nc - 1]);
                vector<int> tri_id;
                tri_id.push_back( a-1 );
                tri_id.push_back( b-1 );
                tri_id.push_back( c-1 );
                tri_ids.push_back(tri_id);
                /*
                printf("Triangle:\n");
                pts[a-1].print(); cout << endl;
                pts[b-1].print(); cout << endl;
                pts[c-1].print(); cout << endl;
                printf("Norm:\n");
                mesh.n.print(); cout << endl;
                */

                //meshes.push_back(mesh);
            }
        }
        /* We calculate our norm here. */
        vector<V3> norms = vector<V3>(pts.size());
        for (int i = 0; i < tri_ids.size(); ++i)
        {
            int a = tri_ids[i][0],
                b = tri_ids[i][1],
                c = tri_ids[i][2];
            Triangle mesh = Triangle(pts[a], pts[b], pts[c], 0, 0, 0);
            norms[a] += mesh.n;
            norms[b] += mesh.n;
            norms[c] += mesh.n;
        }
        for (int i = 0; i < norms.size(); ++i)
            norms[i] = unit(norms[i]);
        for (int i = 0; i < tri_ids.size(); ++i)
        {
            int a = tri_ids[i][0],
                b = tri_ids[i][1],
                c = tri_ids[i][2];
            Triangle mesh = Triangle(pts[a], pts[b], pts[c], norms[a], norms[b], norms[c]);
            meshes.push_back(mesh);
        }
    }

    void Trans(const V3 &v, ld scale) {
        bbox.reset();
        for (auto it = meshes.begin(); it != meshes.end(); ++it)
        {
            it->a = it->a * scale + v;
            it->b = it->b * scale + v;
            it->c = it->c * scale + v;
            (*it) = Triangle(it->a, it->b, it->c, it->na, it->nb, it->nc);
            /*
            printf("Triangle:\n");
            it->a.print(); cout<<endl;
            it->b.print(); cout<<endl;
            it->c.print(); cout<<endl;
            */
            bbox.update(it->a);
            bbox.update(it->b);
            bbox.update(it->c);
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
        //bool ans = tree.brutehit(tree.root, r, t, obj);
        if (!ans)
            return 0;
        V3 x = r.pos + r.dir * t;
        n = obj->calc_norm(x);
        return 1;
    }
};

#include <vector>
#include <cmath>

struct poly
{
    vector<ld> a;

    poly() {
        a.clear();
    }
    poly(ld c) {
        a.clear();
        a.push_back(c);
    }
    poly(ld a1, ld a0) {
        a.clear();
        a.push_back(a0);
        a.push_back(a1);
    }
    poly(const vector<ld> &a): a(a) {
    }

    poly operator +(const poly &p) const {
        vector<ld> ans;
        int size = max(a.size(), p.a.size());
        for (int i = 0; i < size; ++i)
            if (i < a.size() && i < p.a.size())
                ans.push_back(a[i] + p.a[i]);
            else if (i < a.size())
                ans.push_back(a[i]);
            else
                ans.push_back(p.a[i]);
        return ans;
    }
    poly operator -(const poly &p) const {
        vector<ld> ans;
        int size = max(a.size(), p.a.size());
        for (int i = 0; i < size; ++i)
            if (i < a.size() && i < p.a.size())
                ans.push_back(a[i] - p.a[i]);
            else if (i < a.size())
                ans.push_back(a[i]);
            else
                ans.push_back(-p.a[i]);
        return ans;
    }
    poly operator *(const poly &p) const {
        vector<ld> ans = vector<ld>(a.size() + p.a.size() - 1);
        for (int i = 0; i < a.size(); ++i)
            for (int j = 0; j < p.a.size(); ++j)
                ans[i + j] += a[i] * p.a[j];
        return ans;
    }
    ld eval(ld x) const {
        ld f = 0, e = 1;
        for (int i = 0; i < a.size(); ++i)
            f += a[i] * e, e *= x;
        return f;
    }
    ld d(ld x) const {
        vector<ld> dp;
        for (int i = 1; i < a.size(); ++i)
            dp.push_back(a[i] * i);
        return poly(dp).eval(x);
    }
    ld dp() const {
        vector<ld> dp;
        for (int i = 1; i < a.size(); ++i)
            dp.push_back(a[i] * i);
    }
    ld newton_solver(ld x, int dep = 0) const {
        if (dep > 20)
            return D_INF;
        ld fx = this->eval(x);
        if (fabs(fx) < D_EPS)
            return x;
        ld k = this->d(x),
           nx = x - fx / k;
        return this->newton_solver(nx, dep + 1);
    }
};

struct cubic_bezier
{
    ld x[4], y[4];
    poly fx, fy;

    cubic_bezier() {}
    cubic_bezier(const ld *xx, const ld *yy) {
        memcpy(x, xx, sizeof(x));
        memcpy(y, yy, sizeof(y));
        //TODO
        //parameteric polynomial initialization.
        //
        fx = poly(-1, 1) * poly(-1, 1) * poly(-1, 1) * x[0] +
             poly(-1, 1) * poly(-1, 1) * poly(1, 0) * x[1] * 3 +
             poly(-1, 1) * poly(1, 0) * poly(1, 0) * x[2] * 3 +
             poly(1, 0) * poly(1, 0) * poly(1, 0) * x[3];
        fy = poly(-1, 1) * poly(-1, 1) * poly(-1, 1) * y[0] +
             poly(-1, 1) * poly(-1, 1) * poly(1, 0) * y[1] * 3 +
             poly(-1, 1) * poly(1, 0) * poly(1, 0) * y[2] * 3 +
             poly(1, 0) * poly(1, 0) * poly(1, 0) * y[3];
    }
};

struct my_bezier_obj
{
    cubic_bezier b;
    V3 pos, col;
    ld scale;
    MaterialType type;

    my_bezier_obj() {}
    my_bezier_obj(V3 pos, ld scale, const V3 &col, MaterialType type): pos(pos), scale(scale), col(col), type(type) {
        const ld x[] = {0, 7, 0, 0},
                 y[] = {0, 3, 5, 15};
        b = cubic_bezier(x, y);
    }

    bool intersect(Ray r, ld &t, V3 &n) const {
        r.pos = (r.pos - pos) / scale;
        if (fabs(r.dir.y) < D_EPS)
            r.dir.y = (r.dir.y > 0) ? D_EPS : -D_EPS,
            r.dir = unit(r.dir);
        poly fx = (b.fy - r.pos.y) * (1 / r.dir.y * r.dir.x) + r.pos.x,
             fz = (b.fy - r.pos.y) * (1 / r.dir.y * r.dir.z) + r.pos.z;
        /*
        for (int i = 0; i < fx.a.size(); ++i)
            printf("%lf ", fx.a[i]);
        puts("");
        */
        poly ft = fx * fx + fz * fz - b.fx * b.fx;
        //poly ft = fx - b.fx;
        /*
        cout << (b.fy.eval(0.653356) - r.pos.y) * (1 / r.dir.y * r.dir.x) + r.pos.x << endl;
        cout << fx.eval(0.653356) << " " << b.fx.eval(0.653356) << endl;
        */
        t = D_INF;

        ld t0 = 0, tt = 0;
        int IT = 100;
        for (int i = 0; i < IT; ++i)
        {
            t0 = (ld)i / IT;
            t0 = ft.newton_solver(t0);
            if (t0 > 0 && t0 < 1)
            {
                ld y0 = b.fy.eval(t0),
                   t1 = (y0 - r.pos.y) / r.dir.y;
                assert( fabs( ft.eval(t0) ) < D_EPS );
                if (t1 > 0 && t1 < t)
                    t = t1, tt = t0;
            }
        }

        if (!(t < D_INF))
            return false;
        //cout << t << " " << tt << endl;
        V3 o = r.pos + r.dir * t;
        //o.print(), cout << endl;
        ld x = b.fx.eval(tt);
        //cout << x << " " << (b.fy.eval(tt) - r.pos.y) / r.dir.y * r.dir.x + r.pos.x << endl;
        //assert(fabs(sqr(o.x) + sqr(o.z) - sqr(x)) < D_EPS);
        n = V3(b.fx.d(tt), b.fy.d(tt), 0);
        swap(n.x, n.y);
        n.y = -n.y;
        n.z = n.x / x * o.z;
        n.x = n.x / x * o.x;
        n = unit(n);
        return true;
    }
};

#endif
