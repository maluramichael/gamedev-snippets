# curved-paths

A road needs two things from its centre line: an edge at a fixed distance on both sides, and a way to drive along it at a
constant speed. Three handles define the line: P0, a direction handle C and P1. As a quadratic Bezier curve it passes through
P0 and P1 and is pulled towards C. As a circular arc it starts at P0 heading towards C and ends at P1.

Orange is the easy way: shift the control points for the edges, step the curve parameter t evenly for the car and the dots.
Teal is the way that holds up: offset along the normal at every t, and step the distance evenly (through a length table on
the Bezier curve, directly on the arc). The panel shows the numbers: how far the edges are off the wanted width, and the
speed range of the car that steps t. On the arc both ways give the same road, because the offset of an arc is an arc and
angle times radius is the distance. The chips switch the curve, the number of segments N for the Bezier length table and
the sampled edge, and the road width. Drag the handles to a tight turn and a wide road to see the inner edge fold or an arc
collapse. If P1 lies behind P0 no arc fits, and the demo says so. The geometry is in `logic.hpp` and covered by `test.cpp`.
