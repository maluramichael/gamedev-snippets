# point-in-polygon

The classic test whether a point lies inside a polygon, drawn step by step. The mouse cursor is the test point, a ray
runs from it to the right and every edge it crosses is marked. Switch between ray casting (even-odd: an odd number of
crossings means inside) and the winding number (each crossing counts +1 or -1 by edge direction, a sum other than 0
means inside). The pentagram is where they disagree: its middle is outside for even-odd and inside for nonzero. Corners
can be dragged, a click on an edge adds one, right click or Del removes one (three stay). "Snap ray to corner" puts
the ray exactly through a corner, the case that `(a.y > p.y) != (b.y > p.y)` exists for. The logic is in `logic.hpp`
and covered by `test.cpp`.

Write-up: https://malura.de/en/blog/point-in-polygon
