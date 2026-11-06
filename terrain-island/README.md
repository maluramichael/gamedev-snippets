# terrain-island

Turns fractal noise into an island. Noise alone fills the whole map, so a falloff (0 in the middle, 1 at the edge, from a
circle, square or diamond distance and a curve exponent) is combined with it, either by subtracting or by lerping towards
the mask 1 - f. Everything below the sea level is water. The map is on the left, a profile of one row (noise, mask, result
and the sea line) on the right, and the view chips show the noise and the falloff on their own. Sliders set sea level,
falloff strength and curve. With lerp and strength 1 only the cone is left, with strength 0 the land runs off the edge of
the map, both are called out in the panel. The noise is a small gradient noise written for this demo. The logic is in
`logic.hpp` and covered by `test.cpp`.
