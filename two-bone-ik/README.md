# two-bone-ik

An arm of two segments follows the mouse without iterating. Root, joint and target are the corners of a triangle whose sides
are the two segment lengths and the distance from root to target, so the law of cosines gives the angles directly. The
teal ring is everything the end can reach, from |d1 - d2| to d1 + d2, with a hole in the middle when the segments differ.
The two circles around root and target meet in the two knee positions, the second one is drawn as an open ring (chips 1
and 2 pick the side). Outside the ring the formula would take acos of a number beyond 1: "Naive" shows the NaN that
follows, "Checked" stretches or folds the arm towards the target instead. The target runs a demo path until the mouse
moves. The math is in `logic.hpp` and covered by `test.cpp`.
