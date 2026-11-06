# terrain-erosion

A smooth heightmap from noise (left) next to the same map after thousands of drops of rain (right). Each drop rolls
downhill, picks up sediment where it is steep and fast, and puts it down where it slows or has to climb. The sum of
all heights stays the same, only its distribution changes: ridges stay, valleys branch, lowlands fill. A few recent
drop paths are drawn in blue. Below the maps a cut through both along the marked row shows what was taken (orange) and
what was put down (teal). Hover a map to move the row.

The chips show the typical mistake next to the real thing. "Naive" digs the same amount in every step and never puts
anything down, so it leaves trenches, empty hollows and loses material (the figure in the box turns orange). "Sediment"
carries what it digs and drops it again. "Thermal" lets slopes steeper than the talus angle slide, "Lines" draws contour
lines, "Diff" tints the right map by what changed. Dragging on the right map rains on that spot. Sliders set the
drops per second, the inertia (how much a drop keeps its direction over small bumps), the capacity factor, the share of
the free capacity that is dug per step, the evaporation and the talus angle. Keys: Space rain, 1 naive, 2 sediment,
T thermal, C lines, D diff, N new terrain, R reset, Up/Down rate.

The drop count is a rate and runs through an accumulator, so the erosion speed does not depend on the frame rate.
The noise, the drop, the thermal pass and the statistics are in `logic.hpp` and covered by `test.cpp`.
