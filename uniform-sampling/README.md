# uniform-sampling

Uniform random points in a disc, a triangle and on a sphere, naive next to correct. Both sides get the very same random
numbers, only the formula differs: the naive disc takes the radius straight from the random number and piles up in the
middle, the naive triangle piles up at the apex, the naive sphere at the poles. The bars count the points in ten bands
of equal area (rings, stripes parallel to the base, zones of equal height), so "flat" means uniform and the label under
them gives the smallest and largest band as a multiple of the expected count. The correct side uses `sqrt` for the disc
and the triangle (Turk's method 1, or his fold with the F chip) and z = 1 - 2u for the sphere (Archimedes). Chips pick
the shape, 1k, 5k or 20k points and a new seed; the sphere turns by itself and can be dragged. The mappings, the binning
and the sphere camera are in `logic.hpp` and covered by `test.cpp`.
