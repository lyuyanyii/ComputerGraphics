#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <cstring>
#include "bitmap.h"
#include "graphics.h"
#include "hitpoint.h"
#include "object.h"
#include "qmc.h"
//#include <chrono>
#include <vector>
#include <list>
#include <assert.h>
using namespace std;

#define sqr(_) ((_)*(_))

typedef double ld;

static const ld ALPHA = 0.7;

BBox hpbbox;
list<HitPoint*> hitpoints;
vector< list<HitPoint*> > hash_grid;
ld hash_s;
SphereObject sph[] = 
{
    // Scene: radius, position, color, material
    SphereObject(1e5,  V3( 1e5 + 1,   40.8,       81.6 ), V3( 0.99, 0.01, 0.01 ),  MaterialType::Matte ),   //Right
    SphereObject(1e5,  V3(-1e5 + 99,  40.8,       81.6 ), V3( 0.01, 0.01, 0.99 ),  MaterialType::Matte ),   //Left
    SphereObject(1e5,  V3( 50,        40.8,        1e5 ), V3( 0.75, 0.75, 0.75 ),  MaterialType::Matte ),   //Back
    SphereObject(1e5,  V3( 50,        40.8, -1e5 + 170 ), V3( 0.0,  0.0,  0.0  ),  MaterialType::Matte ),   //Front
    SphereObject(1e5,  V3( 50,         1e5,       81.6 ), V3( 0.75, 0.75, 0.75 ),  MaterialType::Matte ),   //Bottomm
    SphereObject(1e5,  V3( 50, -1e5 + 81.6,       81.6 ), V3( 0.75, 0.75, 0.75 ),  MaterialType::Matte ),   //Top
    SphereObject(16.5, V3( 27,        16.5,         47 ), V3( 0.25, 0.85, 0.25 ),  MaterialType::Mirror),   //Mirror
    SphereObject(16.5, V3( 73,        16.5,         88 ), V3( 0.99, 0.99, 0.99 ),  MaterialType::Glass ),   //Glass
    SphereObject(8.5, V3( 50, 8.5, 60 ), V3( 0.75, 0.75, 0.75 ), MaterialType::Matte ), //Middle
};

inline unsigned int get_hash( const int ix, const int iy, const int iz )
{
    return (unsigned int) (
                (ix * 73856093) ^
                (iy * 19349663) ^
                (iz * 83492791)) % hash_grid.size();
}

void build_hash_grid( const int w, const int h )
{
    V3 size = hpbbox.maxi - hpbbox.mini;
    ld irad = dot(size, 1) / 3 / ((w + h) / 2) * 2;

    hpbbox.reset();
    int cnt = 0;
    for (auto it = hitpoints.begin(); it != hitpoints.end(); ++it)
    {
        auto hp = *it;
        hp -> r2 = sqr(irad);
        hp -> N = 0;
        hp -> flux = V3();

        ++cnt;
        hpbbox.update( hp->pos - irad );
        hpbbox.update( hp->pos + irad );
    }

    hash_s = 1.0 / (irad * 2);

    hash_grid.resize( cnt );
    hash_grid.shrink_to_fit();
    for (auto it = hitpoints.begin(); it != hitpoints.end(); ++it)
    {
        auto hp = *it;
        V3 a = ((hp->pos - irad) - hpbbox.mini) * hash_s,
           b = ((hp->pos + irad) - hpbbox.mini) * hash_s;

        for (int iz = abs(int(a.z)); iz <= abs(int(b.z)); ++iz)
            for (int iy = abs(int(a.y)); iy <= abs(int(b.y)); ++iy)
                for (int ix = abs(int(a.x)); ix <= abs(int(b.x)); ++ix)
                {
                    int hv = get_hash( ix, iy, iz );
                    hash_grid[ hv ].push_back(  hp );
                }
    }
}

inline bool intersect(const Ray &r, double &t, int &id)
{
    int n = sizeof(sph) / sizeof(sph[0]);
    ld d = D_INF;
    t = D_INF;
    for (int i = 0; i < n; ++i)
    {
        d = sph[i].intersect(r);
        if (d < t)
            t = d, id = i;
    }
    return (t < D_INF);
}
void trace(const Ray &r, int dep, bool m, const V3 &flux, const V3 &adj, int idx)
{
    /*
    printf("%d, pos = (%.3lf, %.3lf, %.3lf), dir = (%.3lf, %.3lf, %.3lf)\n", 
            dep, r.pos.x, r.pos.y, r.pos.z, r.dir.x, r.dir.y, r.dir.z);
    */
    
    ld t;
    int id;

    ++dep;
    if (!intersect(r, t, id) || dep >= 20)
        return ;

    const auto &obj = sph[id];
    V3 x = r.pos + r.dir * t, n = unit(x - obj.pos), f = obj.col, 
        nl = (dot(n, r.dir) < 0) ? n : n*-1;
    ld p = max(max(f.x, f.y), f.z);

    if ( obj.type == MaterialType::Matte )
    {
        if (m)
        {
            // camera ray: new a hitpoint
            HitPoint* hp = new HitPoint;
            hp->f = f * adj;
            hp->pos = x;
            hp->n = n;
            hp->idx = idx;
            hitpoints.push_back( hp );

            // update the bounding box
            hpbbox.update( x );
        }
        else
        {
            V3 hh = (x - hpbbox.mini) * hash_s;
            int ix = abs(int(hh.x)), iy = abs(int(hh.y)), iz = abs(int(hh.z));
    
            // Lock should be acquired here, we ignore it
            // since collusion rarely happens.
            {
                auto lis = hash_grid[ get_hash(ix, iy, iz) ];
                for (auto it = lis.begin(); it != lis.end(); ++it)
                {
                    auto hp = *it;
                    V3 v = hp->pos - x;

                    if ((dot(hp->n, n) > 1e-3) && (dot(v, v) <= hp->r2))
                    {
                        ld g = (hp->N * ALPHA + ALPHA) / (hp->N * ALPHA + 1);
                        hp->r2 = hp->r2 * g;
                        hp->N ++;
                        hp->flux = ( hp->flux + hp->f * flux / D_PI ) * g;
                    }
                }
            }

            // sampling next direction by QMC

            ld r1 = 2.0 * D_PI * halton( dep * 3 - 1, idx ),
               r2 = halton( dep * 3 + 0, idx ),
               r2s = sqrt( r2 );
            V3 w = nl,
               u = unit(det((fabs(w.x) > .1 ? V3(0, 1, 0) : V3(1, 0, 0)), w)),
               v = det(w, u),
               d = unit( u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2) );

            if (halton(dep * 3 + 1, idx) < p)
                trace( Ray(x, d), dep, m, f * flux * (1.0 / p), f * adj, idx );
        }
    }
    else if ( obj.type == MaterialType::Mirror )
    {
        trace( Ray(x, reflect(r.dir, n)), dep, m, flux * f, adj * f, idx );
    }
    else if ( obj.type == MaterialType::Glass )
    {
        Ray lr( x, reflect( r.dir, n ) );
        bool into = (dot(n, nl) > 0);
        ld nc = 1.0, nt = 1.5, nnt = (into) ? nc / nt : nt / nc,
           ddn = dot( r.dir, nl ), cos2t = 1 - sqr(nnt) * (1 - sqr(ddn));

        // total internal reflection
        if (cos2t < 0)
            return trace( lr, dep, m, f * flux, f * adj, idx );
        
        V3 td = unit( r.dir * nnt - n * ( (into ? 1 : -1) * ( ddn * nnt + sqrt( cos2t )) ));
        ld a = nt - nc, b = nt + nc, R0 = sqr(a) / sqr(b), c = 1 - (into ? -ddn : dot(td, n)),
           Re = R0 + (1 - R0) * c * c * c * c * c, P = Re;
        Ray rr(x, td);
        V3 fflux = f * flux, fadj = f * adj;

        if ( m )
        {
            //camera ray: trace both directions
            trace( lr, dep, m, fflux, fadj * Re, idx );
            trace( rr, dep, m, fflux, fadj * (1.0 - Re), idx );
        }
        else
        {
            if (halton( dep * 3 - 1, idx ) < p)
                trace(lr, dep, m, fflux, fadj * Re, idx);
            else
                trace(rr, dep, m, fflux, fadj * (1.0 - Re), idx);
        }
    }
    else
        assert(false); //Undefined Material Type
}

void trace_ray( int w, int h )
{
    Ray cam(
        V3(50, 48, 295.6),
        unit(V3(0, -0.042612, -1))
        );
    
    V3 cx = V3( w * 0.5135 / h, 0, 0), cy = unit( det( cx, cam.dir ) ) * 0.5135;

    for (int y = 0; y < h; ++y)
    {
        fprintf( stdout, "\rHitPointPass %5.2f%%", 100.0 * (y + 1) / h );
        for (int x = 0; x < w; ++x)
        {
            int idx = x + y * w;
            V3 dir = cx * ((x + 0.5) / w - 0.5) + cy * (-(y + 0.5) / h + 0.5) + cam.dir;
            trace( Ray(cam.pos + dir * 140, unit(dir)), 0, true, V3(), 1, idx );
        }
    }
    
    fprintf( stdout, "\n" );
    fprintf( stdout, "Ray trace finished.\n");

    build_hash_grid( w, h );

    fprintf( stdout, "Hash grid built.\n");
}

void gen_photon( Ray *r, V3 *f, int idx )
{
    *f = 2500 * D_PI * 4;
    ld p = 2.0 * D_PI * halton( 0, idx ),
       t = 2.0 * acos( sqrt( 1.0 - halton( 1, idx ) )),
       sint = sin( t );

    r->dir = V3( cos(p) * sint, cos(t), sin(p) * sin(t) );
    r->pos = V3(50, 60, 85);
}

void trace_photon( int s )
{
    #pragma omp parallel for schedule(dynamic, 1)
    for (int i = 0; i < s; ++i)
    {
        ld p = 100.0 * (i + 1) / s;
        fprintf( stdout, "\rPhotonPass %5.2f%%", p);

        Ray r;
        V3 f;
        for (int j = 0; j < 1000; ++j)
        {
            gen_photon( &r, &f, 1000 * i + j );
            trace( r, 0, false, f, 1, 1000 * i + j );
        }
    }

    fprintf( stdout, "\n" );
    fprintf( stdout, "Photon trace finished" );
}

void density_estimate( V3* col, int s )
{
    for (auto it = hitpoints.begin(); it != hitpoints.end(); ++it)
    {
        auto hp = *it;
        int i = hp->idx;
        col[i] += hp->flux * (1.0 / (D_PI * hp->r2 * s * 1000.0));
    }
}

int main()
{
    int w = 1280, h = 1080, s = 10000;
    V3 *col = new V3[w * h];

    hpbbox.reset();

    trace_ray( w, h );
    trace_photon( s );
    density_estimate( col, s );
    
    save_to_bmp( "image.bmp", w, h, &col[0].x, 2.2 );

    return 0;
}

