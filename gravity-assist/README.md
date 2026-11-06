# gravity-assist

A probe flies past a planet that moves at a constant speed. Seen from the planet, the probe arrives at some speed,
swings around and leaves at exactly that speed, only the direction has changed. Seen from the sun (the frame in which
the planet moves) the same flight ends faster or slower, because the planet's velocity V is added to both the
incoming and the outgoing velocity: v = V + u. Passing behind the planet speeds the probe up, passing in front slows it
down, and the gain can never exceed 2 V. The sun itself is not simulated: over the few seconds of an encounter its pull is
the same on probe and planet, so it cancels. The planet flies in a straight line, the probe is a test mass that feels
only the planet (velocity Verlet, 5 ms steps). The whole flight is computed in one go and then only played back, so the
dashed prediction while you drag and the animation afterwards are the same data. The trail is coloured by the speed in
the frame you look from. The table reads the speeds where the probe crosses the dashed zone circle, in and out, in both
frames; the diagram below it adds the vectors.

Press and drag in space to set the start and the velocity (the prediction follows the mouse), a plain click starts the
probe at rest in the frame on screen. 1 2 3 load a pass behind the planet, a pass in front and a start from rest, F
switches between the sun's and the planet's point of view (the stage slides), V cycles the planet's speed (0 to 90), M
the planet's mass, R replays, Space pauses, the bar under the stage scrubs, right click or Esc cancels a drag. A launch is
stored in the planet's frame, so changing the planet's speed changes only what the sun sees. Planet speed 0 shows no
gain at all. The logic (`logic.hpp`) has no SDL dependency and is covered by `test.cpp`, including the turn angle
against the closed-form hyperbola and the textbook 90 degree case.

See also [orbit-mechanics](../orbit-mechanics) and [n-body-gravity](../n-body-gravity).
