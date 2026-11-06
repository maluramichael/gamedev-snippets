# terrain-fbm

Terrain from noise, drawn two ways side by side. Left is value noise (one random number per lattice point), right is
gradient noise (one random direction per lattice point), both summed over several octaves (fractal Brownian motion)
from the same settings. Value noise shows its grid as blocks and diamonds, gradient noise hides it. Sliders set octaves,
lacunarity (frequency factor per octave), persistence (amplitude factor per octave), base frequency and seed. Dragging a
map pans the view. The boxes below show the octave layers of the gradient noise one by one: a single layer looks like
clouds, only the sum looks like land. The chips switch the fade curve (linear shows the grid in both noises, cubic
3t^2 - 2t^3, quintic 6t^5 - 15t^4 + 10t^3), the terrain coloring (off: plain height as gray) and the normalization (off:
the sum leaves 0..1 and the part above 1 turns orange). Keys: 1 2 3 fade, T terrain, N normalize, R reset, Up/Down
octaves, Left/Right seed. The noise, the octave sum and the colors are in `logic.hpp` and covered by `test.cpp`.
