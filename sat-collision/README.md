# sat-collision

The separating axis theorem for two convex polygons. Drag and turn the shapes (mouse wheel or Q/E turns the one with the
thick outline). Every edge normal is an axis, the list on the right shows the overlap or gap of each. The selected axis
is drawn as a line with the shadow of A (teal) and B (orange) on it. A gap on any axis proves the shapes do not touch,
overlap on all of them means collision, and the smallest overlap is the shortest way out: the arrow is the minimum
translation vector (MTV), the dashed outline is B after the push (Space applies it). Two chips switch the usual
mistakes on: "Normalize axes" off uses axes as long as the edge (the yes/no answer stays right, the MTV does not),
"Orient MTV" off keeps the direction of the axis instead of the side B has to leave through (the push goes into A).
Arrow keys or the list pick the axis, 1 to 3 change the pair of shapes, R resets. The logic is in `logic.hpp` and
covered by `test.cpp`. Convex polygons only.
