# wfc-tiles

Wave function collapse in its simple tiled form, drawn step by step. A map of 30x11 cells is built from 29 tiles
(grass, water, shores, roads, bridges) that are drawn by code. Each tile has an edge label per side, two tiles may
touch only if the labels on the touching edges are equal. Every cell starts as "any tile" and shows a washed-out
blend of the tiles still possible. The solver collapses the cell with the lowest entropy to one tile (chosen by
weight) and propagates the consequences to the neighbours; orange frames mark the cells waiting for that check.
Click a cell to collapse it yourself. The speed chips run the solver at 15, 100 or 600 steps per second, or as fast
as one frame allows (at most 5000 steps per frame).

Switch the cell order to "random" and contradictions appear: a cell with no tile left. Then either the whole map
restarts or, with backtracking, the last decision is undone and its tile is banned. Both are capped (40 restarts,
200 backtracks per attempt), after that the demo reports "gave up". "Compare orders" runs 40 maps per order and
counts the contradictions. "Tuned" weights make grass common and roads rare, "Uniform" gives all 29 tiles the same
chance. Hovering a cell lights up the tiles it can still become in the strip below the map. The solver is in
`logic.hpp`, the tile pictures in `art.hpp`, both are covered by `test.cpp`.
