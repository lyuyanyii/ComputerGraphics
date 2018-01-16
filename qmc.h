#ifndef __QMC_H__
#define __QMC_H__

typedef double ld;

int primes[61] = {
    2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79,
    83, 89, 97, 101, 103, 107, 109, 113, 127, 131, 137, 139, 149, 151, 157, 163, 167, 173, 179, 181,
    191, 193, 197, 199, 211, 223, 227, 229, 233, 239, 241, 251, 257, 263, 269, 271, 277, 281, 283
};

inline int rev(const int i, const int p)
{
    if (i == 0)
        return i;
    
    return p - i;
}

ld halton(const int b, int j)
{
    const int p = primes[b];
    ld h = 0, f = 1 / (ld)p, fct = f;

    while (j > 0)
    {
        h += rev(j % p, p) * fct;
        j /= p;
        fct *= f;
    }
    return h;
}

#endif
