# camera-smoothing

Three cameras chase the same scripted player: A moves a fixed fraction of the gap per frame (`lerp(cam, target, 0.1)`),
B derives that fraction from the frame time (`1 - exp(-lambda * dt)`), C is a critically damped spring with a velocity.
`lambda` is chosen so that A and B are the same camera at 60 fps. Switch the simulated frame rate between 15, 60 and 144
fps and A changes completely while B and C hardly move. The scope on the right plots the gap or the camera speed over the
last four seconds. The two teleports of the player show the difference between B and C: exp decay starts at full speed,
the spring accelerates. The logic is in `logic.hpp` and covered by `test.cpp`.

Keys: `1` `2` `3` frame rate, `S` scope (gap or speed), `R` restart.
