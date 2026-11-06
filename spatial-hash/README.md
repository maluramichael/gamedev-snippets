# spatial-hash

2000 circles bounce around and every step needs the pairs that touch. Three ways to find them: brute force (every pair,
n(n-1)/2 tests), a uniform hash grid (only pairs that share a cell) and a quadtree (only pairs in the same or an enclosing
node). The number that matters is the pair-test count, it is the same on every machine. The milliseconds next to it are
measured in your browser and only a footnote. Hover the field to see which cell (or node) a circle falls into. The slider
sets the grid's cell size (4 to 128 px, also Left/Right): the test count falls with smaller cells, the cell entries
rise, and the sum has its minimum a bit above the circle diameter (about 12 px for the 7 px circles). The big circles span several cells, so a pair can show
up in more than one cell. "Dedup" (D) keeps it only in the first cell both share, switch it off to see the duplicates in
the test count and in the contacts. Keys 1 to 3 pick the method, 4 to 7 the number of bodies, B toggles the big circles,
R restarts the same scene.

The structures and their edge cases (touching, circles outside the world, a cluster that cannot be split) are in
`logic.hpp` and covered by `test.cpp`; `spatial-hash-logic --report` prints the pair-test numbers of the start scene.
The grid is a bounded array indexed by cell, for an endless world you would put the cell coordinates through a real hash.
