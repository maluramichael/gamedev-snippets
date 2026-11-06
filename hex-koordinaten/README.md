# hex-koordinaten

One hex grid, four ways to number it. The cursor cell is shown as offset (what you would store in an array), axial and
cube (what you do the math in) and doubled coordinates. Click or drag to place the cursor, the six orange neighbors and
the distance heatmap follow. Hovering a cell shows the cube difference and the distance as the largest of its three
absolute values, next to the tempting wrong answer (columns plus rows on the offset grid, chip "Naive"). Chips switch
pointy/flat and odd/even (the four offset conventions) and what the cells show. Q W E A D Z X C or the arrow keys
(Shift picks the other side) move the cursor, the offset neighbor tables are used for that, so they have to agree
with the orange cube neighbors. The conversions, distance, neighbors and the hex-to-pixel formulas are in `logic.hpp`
and covered by `test.cpp`. The hex under the mouse is found with a nearest-center scan, the direct formula is left
for a later post.
