# line-circle-drawing

Which cells does a line or a circle cover on a grid? Drag the handles A and B. In line mode three algorithms answer
for the same line: lerp (evenly spaced points, rounded), Bresenham (error term) and supercover (every cell the line
touches, with a choice for lines that pass exactly through a corner). Walls (right click or W) turn the difference
into a line of sight check: clear for one algorithm, blocked for another. In circle mode A is the center and B sets
the radius. The distance test, the midpoint circle (one octant, mirrored eight times) and the tempting "left and
right cell per row" version are drawn side by side, the last one leaves gaps at the top and bottom. The logic is in
`logic.hpp` and covered by `test.cpp`, including an independent exact check of the supercover cells.
