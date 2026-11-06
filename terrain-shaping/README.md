# terrain-shaping

The same noise, reshaped. A fractal sum of gradient noise gives a smooth height field and, from the same octaves, a ridged
one. Between the noise and the picture sit three small functions: a mix that adds the ridge detail on the summits only, a
monotone height curve with five draggable nodes (presets: linear, valleys = x squared, S-curve) and optional terraces
with sloped risers. The map is a shaded relief with a sea level, the histogram shows the heights before the curve
(outline) and after everything (filled), the cross-section lays the raw noise over the shaped line along one row. "Naive"
puts the ridges in the valleys too and turns the terrace risers into vertical cliffs, the two usual mistakes. Dragging the
map pans, hovering it picks the cross-section row. Keys: 1 2 3 curve, Space new seed, O octaves, T terraces, S steps,
R ridged, N naive. The noise, the curve, the terraces and the shading are in `logic.hpp` and covered by `test.cpp`.
