# terrain-rivers

Rivers and lakes read off a heightmap instead of drawn. The map is a noise island of 160 x 100 cells. Every cell drains into
its steepest lower neighbor (D8, diagonals count sqrt(2) away), the cells are processed from high to low and each one passes
the rain it collected on, and a river is wherever at least "threshold" cells drain through. On raw noise terrain that ends in
pits, small depressions where the water has no way down: the orange squares, and the rivers stop there. "Fill sinks" raises
every depression to its lowest way out with Priority-Flood (plus a tiny epsilon so flooded cells keep a direction), the raised
cells are the lakes, and every raindrop reaches the sea. Hover over the map to follow one drop. Sliders set the river threshold
and the sea level. Keys: 1 raw, 2 fill sinks, W water on/off, N new terrain, R reset, Left/Right threshold, Up/Down sea level.
The terrain, the fill, the flow directions, the accumulation and the colors are in `logic.hpp` and covered by `test.cpp`.
