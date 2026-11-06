# triangle-rasterizer

A fan of triangles on a magnified pixel grid. A pixel is drawn when its centre is inside a triangle, decided with three
integer edge functions. The question is a centre that lies exactly on an edge two triangles share. "Inclusive" counts it
for both (orange pixels, drawn twice: a bright seam under additive blending), "Exclusive" for neither (dark pixels, holes),
"Top-left" gives it only to the triangle for which the edge is a top or left edge, so every pixel is drawn exactly once.
Corners can be dragged (they only move to places where the fan stays valid), "Snap to centres" puts them on pixel centres,
which makes edges run through centres everywhere, and the side panel counts pixels drawn once, twice and not at all.
The logic is in `logic.hpp` and covered by `test.cpp`, including a property test: any split of a rectangle into triangles
covers each pixel exactly once.
