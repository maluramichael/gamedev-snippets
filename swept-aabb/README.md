# swept-aabb

Two lanes, one box, one wall. The top lane moves the box and then tests for overlap, so a fast box lands behind a thin
wall and nothing is ever detected (tunneling). The bottom lane sweeps the box along the whole frame and gets `t`, the
free fraction of the motion in 0..1, plus the side that was hit. Chips set speed (pixels per frame), angle and wall
thickness and switch between stopping at the wall and sliding along it (S). A click in a lane or up/down sets the start
height, Space restarts. The simulation is slowed down to one frame per 200 ms so the jumps are visible. The logic
(`logic.hpp`) has no SDL dependency and is covered by `test.cpp`: head-on hits, all four sides, zero velocity on one
axis, touching and grazing boxes, corner ties, stop and slide, and the speeds at which the naive test falls through.

See also [fixed-timestep](../fixed-timestep) and [rigid-bodies](../rigid-bodies).
