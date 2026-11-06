# terrain-warp

Domain warping: the terrain height is not read at the point p but at a shifted point, f(p + s * q(p)), where q is a
second noise field. Stage 2 feeds the first shifted point into a second field, r = q(p + s * q), and reads at
p + s * r. The plain map is on the left of the divider, the warped one from the same seed on the right, and the divider
can be dragged. The strength slider (or Left/Right) sets s, the chips pick 0, 1 or 2 stages, switch the comparison,
show the offset of the lookup as arrows (drawn at a third of their length) and start a sweep over s (Animate). Hover the
map: the line leads from the pixel to the point it really reads from, and a cut-out of the plain map shows what is
there. "Folded" is the share of the map where the lookup runs backwards (the Jacobian determinant of the warp is 0 or
negative), the part that makes a strong warp look torn. Keys: 1 2 3 stages, C compare, V arrows, A animate, N new
seed, Left/Right strength. The noise, the warp and the fold measure are in `logic.hpp` and covered by `test.cpp`.

The idea comes from Inigo Quilez, https://iquilezles.org/articles/warp (own code, own words).
