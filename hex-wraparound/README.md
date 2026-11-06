# hex-wraparound

A hexagon-shaped hex map whose edges join up. Copies of a map of radius N, moved by the six mirror centers
(2N+1, -N, -N-1) turned in 60 degree steps, cover the plane without gaps, so a step off the map is a step into the next
copy, and subtracting that copy's center puts the tile back on the map. A lookup table for the 6(N+1) tiles just outside
the map makes a step one addition and at most one table read. The figure walks on its own, `Plane` draws the map with its
six copies and the mirror centers m0 to m5, `Fringe` only the ring of copy tiles around the map, labeled with the tile of
the real map each one stands for. Hovering a tile lights it up in every copy, a click puts the figure there. Orange hexes
mark tiles the figure has been on (on a map with a prime number of tiles, 7, 19, 37 or 61, a straight line visits all of
them before it closes). `Naive` jumps to the tile opposite the one you left instead: the walk then never leaves its row
and edge tiles have fewer than six different neighbors. Keys: 1 to 3 radius, 4 5 view, 6 7 rule, Space walk, R reset,
Q E A D Z C one step. The math (mirror centers, wrap, lookup table, naive jump, pixel positions) is in `logic.hpp` and
checked by `test.cpp` against an independent lattice test. Only pointy hexes are drawn.
