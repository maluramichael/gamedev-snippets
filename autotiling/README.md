# autotiling

Paint walls and grass with the mouse and every cell picks its own tile from a bit mask of its eight neighbours. The
four chips at the bottom switch the way the tile is chosen. 4-bit only looks at north, east, south and west and cannot
draw an inner corner. 8-bit raw looks at all eight neighbours and needs 256 tiles. 8-bit with the corner rule counts a
diagonal neighbour only when both of its edge neighbours are set, which draws the same picture with 47 tiles. The dual
grid moves the tiles onto the cell corners and gets by with 16. The atlas on the right shows all tiles of the current
mode and lights up the ones the map uses; the card below it shows the mask of the cell under the mouse. Keys: 1-4 mode,
W G E tool, T tile grid, C clear, R reset, right mouse button erases. All tiles are drawn from rectangles, there are no
image files. The logic (masks, corner rule, tile pieces) is in `logic.hpp` and covered by `test.cpp`, including a check
that the 47-tile, the 256-tile and the dual-grid rendering of a random map are identical pixel by pixel.
