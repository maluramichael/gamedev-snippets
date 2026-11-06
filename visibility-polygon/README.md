# visibility-polygon

What a light at the mouse cursor can see, drawn from line segment walls. The lit area is a polygon: one ray at every wall
end plus one just before and one just after it, the hits sorted by angle and joined. Switch to "exact corners" to see the
classic mistake (only the ray through the end: the point behind it is missing and the shadow edge turns into a chord) and
to "uniform rays" to see that 24, 90 or 360 evenly spaced rays do not find the wall ends either (chip N cycles the count).
Wall corners can be dragged (a box stays closed), R shows the rays, F turns the light into a flashlight cone (mouse wheel or
Q/E aims it), S switches the scene, X resets the walls. The geometry is in `logic.hpp` and covered by `test.cpp`, including a
ray that hits a shared wall end exactly and a cross-check of the polygon against a direct line of sight test.

Not handled: a light outside the outermost walls (the demo keeps it inside the field, rays that hit nothing are dropped).
