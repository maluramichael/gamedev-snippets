# flow-field

200 agents walk to one goal on a grid with walls. A flow field does the search once: a breadth first search starts at the
goal and writes the number of steps into every cell (the heat map), every cell gets an arrow that points downhill on that
map, and an agent only reads the arrow of the cell it stands in. Click or drag moves the goal, tool 2 paints and erases
walls, and the search runs again after every change. Chip 6 switches to the alternative, one A* search per agent, and the
card under the grid counts how many cells each side expanded after the last change (the A* number depends on where the
agents stand). Heat map and arrows can be switched off, R respawns the agents, C clears the walls, M resets the map. The
logic (search, arrows, wall collision, A*) is in `logic.hpp` and covered by `test.cpp`. The wave that grows from the goal
after a click is only a drawing, the agents use the finished field at once. Agents do not avoid each other, so they
bunch up in the streams.
