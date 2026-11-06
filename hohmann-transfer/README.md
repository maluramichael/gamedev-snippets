# hohmann-transfer

A ship on a low circular orbit has to meet a target on a higher one. The cheap way takes two short burns along the
direction of travel: the first (dv1 = v1 (sqrt(2 r2 / (r1 + r2)) - 1)) stretches the orbit into an ellipse whose far point
touches the target orbit, the second at that far point (dv2 = v2 (1 - sqrt(2 r1 / (r1 + r2)))) rounds it off again. The
catch is the timing. The ship needs half an ellipse, `pi * sqrt(a^3 / mu)`, to get there, and the target has to arrive at the
far point at the same moment, so at the first burn it must lead the ship by `pi - n2 * T` (the phase angle, about 67 degrees
for the default orbits). The dashed ellipse shows where a burn right now would end, the hollow ring where the target will be
by then; when they meet, the window is open. "At target" points the same fuel straight at the target instead, which ends on
a different orbit and never gets there. The autopilot fires inside a frame at the exact moment, so it is the reference for the
timing a player can only approximate. Click in space or press B to burn (also the chip), A toggles the autopilot, P and T
switch the aim, 1 2 3 pick the target orbit, - and + change the speed (0.25x makes the window easy to hit), R restarts, Space
pauses. The logic (`logic.hpp`) has no SDL dependency and is covered by `test.cpp`, including the closed forms against the
exact orbit propagation.

See also [orbit-mechanics](../orbit-mechanics) and [gravity-assist](../gravity-assist).
