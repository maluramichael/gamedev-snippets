# rigid-bodies

2D rigid body physics with circles and oriented boxes. Contacts come from circle-circle, circle-box and box-box
(separating axis test, reference face, clipping to at most two points) tests, and are resolved with impulses:
restitution along the normal, Coulomb friction along the tangent, a small positional push-out, and eight sequential
impulse iterations with warm starting so stacks stand still. The world runs in fixed steps of 1/120 s from an
accumulator. Click empty space to spawn a body, drag a body with the mouse spring, R clears the scene. The logic
(`logic.hpp`) has no SDL dependency and is covered by `test.cpp`.

See also [delta-time](../delta-time) and [fixed-timestep](../fixed-timestep) for the loop underneath.
