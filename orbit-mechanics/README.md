# orbit-mechanics

One sun, one body, four answers to "where is it after t seconds". The exact Kepler orbit (white) is propagated in
closed form with universal variables, so it has no step size and no drift. Explicit Euler (orange), semi-implicit
Euler (teal) and velocity Verlet (blue) start from the same position and velocity and advance in fixed steps of h
(0.01 to 0.25 s). Explicit Euler spirals outward because it moves along the tangent before it turns the velocity, and
its angular momentum grows by exactly `1 + h^2 mu / r^3` per step. Swapping those two lines (semi-implicit) keeps the
angular momentum to rounding error and the energy bounded. The table shows the drift of energy (dE) and angular
momentum (dL) since the launch and the distance to the exact body. Click in space for a circular orbit, press and drag
to set the launch velocity (the exact ellipse follows the mouse), 1 2 3 load a circle, an ellipse or a comet, R
restarts, Space pauses, - and + change h, E S V show or hide a method, A shows the exact orbit cut into eight sectors
of equal time (equal times, equal areas). The logic (`logic.hpp`) has no SDL dependency and is covered by `test.cpp`,
including a cross-check of the exact orbit against RK4 and against Kepler's equation.

See also [delta-time](../delta-time) and [fixed-timestep](../fixed-timestep) for the loop underneath.
