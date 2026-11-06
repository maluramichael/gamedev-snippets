# hex-pixel-runden

From a mouse position to a hex: pixel to fractional cube coordinates, then rounding. The left card shows the numbers for
the mouse position: the fractional q, r, s, each rounded on its own, how far each one moved, and the correction. Rounding
the three components separately does not keep q + r + s = 0 on about a quarter of the plane (chip "Naive", the orange tiles
mark those places). "Rounded" resets the component that moved the most and always finds a hex. Line mode draws a line
between a start hex (click) and the hex under the mouse: N + 1 samples of a straight lerp in cube space, each rounded;
"Nudge" shifts the whole line by 1e-6 so a sample on the border of two hexes falls clearly to one side. Pointy and flat
layouts use the same code with different matrices. The math is in `logic.hpp` and covered by `test.cpp`.

Keys: 1 Naive, 2 Rounded, 3 Pick, 4 Line, 5 Pointy, 6 Flat, N Nudge.

Idea after the hexagon guide by Red Blob Games (https://www.redblobgames.com/grids/hexagons/), code written from scratch.
