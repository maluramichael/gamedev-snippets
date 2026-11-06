# astar-vergleich

Four ways to find a path on a grid, side by side on the same map: breadth first search, Dijkstra, greedy best first
and A*. They are one loop in `logic.hpp` that only differs in the number that picks the next waiting cell (order of
discovery, g, h, or g + weight * h). Teal cells are expanded, orange cells are the frontier, the dark line is the path.
Paint walls and mud (terrain costs 5), drag the start S and the goal G, switch between 4 and 8 directions (diagonals
may not cut corners) and move the weight slider: 0 turns A* into Dijkstra, 1 is plain A*, anything above 1
overestimates and gives up the cheapest path. The status line compares the cost of the path found with the optimum.
The start map has a bucket that opens towards S (the trap for greedy search) and a mud patch on the straight line
(BFS and greedy walk right through it). Every cell is expanded at most once, closed cells are never reopened. The
logic is covered by `test.cpp`. The idea follows the A* articles of Red Blob Games, the code is written from scratch.
