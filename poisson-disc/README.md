# poisson-disc

The same number of points three ways: uniform random, a jittered grid and Poisson disc sampling after Bridson. Every
point carries a disc of radius r/2, so two points closer than r show as overlapping discs and an orange line, and each
panel counts those pairs. The right panel grows live: a background grid of r/sqrt(2) cells (at most one point per
cell), an active list, and a ring of up to k candidates between r and 2r around the active point being worked on.
The chips change r and k, show the grid and the discs, replay the run and roll a new seed. Painting in the right
panel lowers the local radius (denser points), two points then keep the larger of their two radii apart. The
algorithm, the density map and the measuring are in `logic.hpp` and covered by `test.cpp`.
