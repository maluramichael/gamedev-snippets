# steering-behaviors

An agent that chases, flees from, arrives at, wanders around or follows a path, built on one rule from Craig Reynolds'
GDC 1999 paper: steering = desired velocity - current velocity, capped by a force limit. The arrows show the velocity
(green), the velocity the behavior wants (dark) and the difference between them (orange). Seek keeps wanting full speed,
so it shoots through the target and swings back forever; arrive asks for less speed the closer it gets and stops. Keys
1 to 5 pick the behavior, the mouse (or a click) moves the target, R resets.

Two agents run from the same state, A at 120 steps per second and B at 30. With "dt scaling" on (key D) the force limit is
a velocity change per second and both stay together. Off, the limit is per step and B crawls while A races ahead, the
gap number shows by how much. F switches the force limit off: the velocity snaps to the wish and the agent turns on a dime.
The logic is in `logic.hpp` and covered by `test.cpp`.
