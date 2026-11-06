# hex-astar

A* on a hex grid and on a square grid, side by side. The search in `logic.hpp` never learns which grid it runs on: it
asks for the neighbors of a cell, the cost of entering a cell and the estimated distance to the goal. Switch the grid and
the map stays, only the neighbor function (6 cells instead of 4) and the distance (cube distance instead of
`|dx| + |dy|`) change. Paint walls, forest (cost 3) and swamp (cost 6) with the brush (right click erases), drag the
start A and the goal B. Teal cells were expanded by the search, orange ones are known but not expanded yet, hovering a
cell shows `g + h = f`. "Heuristic" off turns A* into Dijkstra: same path, more cells expanded. Keys: 1 hex, 2 square,
H heuristic, W/F/S/G brush, R reset. The logic (search, neighbors, distance, geometry and picking) is covered by
`test.cpp`.
