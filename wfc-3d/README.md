# wfc-3d

Wave function collapse in three dimensions. A grid of 12x6x12 cells is filled from 34 tiles (air, trees, houses with
upper floors and roofs, wall pillars and round towers) by the same two steps as in the flat version: collapse the cell
with the lowest entropy, then propagate what that rules out. The only change is the neighbour list, which has six
entries instead of four. The rules are plain functions over (tile, direction, tile): wall arms must meet on both sides of
a border, and the vertical rules say what may stand on what. Undecided cells are translucent cubes whose size shows
how likely a solid tile is still to appear there, cells touched by the propagation glow orange, and the cell that is
collapsed next has a frame. Drag orbits the camera, the wheel zooms, a click on a cube collapses that cell.

The chips set the speed (40 and 400 steps per second, 2500, or as many as a frame allows, at most 6000 per frame), the
cell order (entropy, layer by layer from the ground, random) and the reaction to a contradiction: undo the last decision
and ban its tile, or restart the world. Both are capped (40 restarts, 200 undos per attempt). The seed is the world: the
same seed with the same settings builds the same town. The solver is in `logic.hpp`, the tile rules in `castle.hpp`, the
meshes in `mesh.hpp`, all covered by `test.cpp`.
