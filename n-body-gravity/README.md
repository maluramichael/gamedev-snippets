# n-body-gravity

A few bodies pull on each other with Newtonian gravity, and the demo shows what the numbers do. Four scenes: the
figure-eight orbit of three equal masses (Chenciner and Montgomery), a sun with four planets, a cold cluster of 14
bodies and Burrau's Pythagorean problem (masses 3, 4, 5 at rest on a 3-4-5 triangle). Three integrators (explicit
Euler, leapfrog, Runge-Kutta 4), three fixed step sizes and Plummer softening, `a = G m d / (d^2 + eps^2)^1.5`, can be
switched. Energy, its drift against the start and the total momentum are printed live: momentum survives every
integrator because the pair forces are equal and opposite, energy is what tells them apart. Drag in space to launch a
new body, the arrow is one simulated second of flight and the dotted line is where the whole system would carry it.
Changing method, step or softening restarts the scene, so runs can be compared from the same start. Keys: 1 to 4 scene,
M method, S softening, T trails, R reset, space pause. The logic (`logic.hpp`) has no SDL dependency and is covered by
`test.cpp`.

See also [rigid-bodies](../rigid-bodies) for another fixed-step simulation.
