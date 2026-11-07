# wfc-propagation

Wave function collapse with three levels of propagation side by side, on the 29 tiles of `wfc-tiles`. The same seed runs in
three maps of 10x10 cells: no propagation (an open cell still shows "any tile", the truth is only found when the solver stands
at it), neighbors only (the four neighbors of a decided cell drop the tiles that no longer fit) and the full cascade (a neighbor
that lost tiles tells its own neighbors, until nothing changes). A cell with no tile left is a contradiction and the map starts
over, at most 100 times. "X-ray" hatches the cells of the unpropagated map that are already lost but not noticed yet. Click a cell
to decide it, the chips pick the cell order (lowest entropy, reading order, random), the speed and a new map.

The table runs 40 maps for every combination of order and level in small slices over many frames and counts the contradictions
(clean = a map without any). "New seeds" measures again with other seeds. Without propagation the entropy of every open cell is
the same, so "lowest entropy first" degrades to random there. The solver and the benchmark are in `logic.hpp`, the tile pictures
in `art.hpp` (a copy of the one in `wfc-tiles`), `test.cpp` covers the solver with hand-computed cases and the measured claims.
