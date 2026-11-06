# delta-time

Two squares run the same lap. The red one moves a fixed number of pixels per frame, the blue one moves
`speed * delta time`. Click (or press 1 to 4) to switch between 15, 30, 60 and 144 frames per second: the
red square speeds up and slows down with the frame rate, the blue one keeps its speed. Delta time is clamped
to 0.25 s so a stalled tab does not teleport anything.

The fixed timestep variant of the same problem is in [update-loop](../update-loop).
