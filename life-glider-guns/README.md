# life-glider-guns

Conway's Game of Life with a palette of the things that make it interesting: the glider (four directions, four phases), the block,
the blinker and the Gosper gun, which shoots a glider every 30 generations. Click to place a shape, drag with Draw to paint cells,
right click erases. A glider moves one cell diagonally every 4 generations, and where two of them meet depends on their offset: the
Collision chip loads four cases (they miss, both vanish, one block is left, four blocks are left). The graph shows the population,
and while the gun shoots undisturbed it climbs by 5 every 30 generations. The edge is a chip: on a torus the field wraps around,
with walls a glider that arrives ends as a 2x2 block. Moving cells are teal, cells that have lived 8 generations or more are ink.
The rules, the shapes and the glider collisions are in `logic.hpp` and covered by `test.cpp`, whose reference numbers come from a
separate Python implementation.
