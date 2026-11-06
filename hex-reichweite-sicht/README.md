# hex-reichweite-sicht

A unit on a hex map with walls, and two questions about it: how far can it go, and what can it see. Range has two modes:
a loop over q and r that claims every hex within N steps (and ignores walls), and a flood fill that spreads layer by layer
and does not step into walls. The hexes the loop claims and nobody can walk to are shown in orange. Sight has three:
the hexes on a straight line (cube coordinates, rounded), the exact line from center to center tested against the wall
hexagons, and the same with the six corners of the target as extra tries. An orange dot marks a hex whose answer differs
from the center line, hovering a hex draws the lines behind its answer. Click or drag paints walls, dragging the unit (or
a right click) moves it, `-` and `+` change the number of points or the radius. A wall only blocks a line that goes through
its inside: touching a corner or sliding along an edge is not blocking. The logic is in `logic.hpp` and covered by `test.cpp`.
