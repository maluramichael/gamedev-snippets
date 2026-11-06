# grid-raycaster

A Wolfenstein-style raycaster in a grid world, with the map and its rays on the left and the resulting wall columns on
the right. One ray per column finds the first wall cell; the wall is drawn taller the closer it is. Two choices are
switchable. The search: the DDA jumps from grid line to grid line and visits exactly the cells the ray passes through,
a fixed-step march samples along the ray, skips corners that are thinner than its step and reports every wall up to one
step late, which shows as jagged wall edges (the counters show cells per ray for both, and how many rays end in another
cell than the DDA's). The distance: using the straight length of the ray for the wall height bends flat walls (fisheye),
using the distance along the view direction keeps them flat. Click a cell to toggle a wall, drag the dot to move, drag in
the view to turn. The logic is in `logic.hpp` and covered by `test.cpp`.
