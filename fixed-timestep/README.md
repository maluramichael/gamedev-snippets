# fixed-timestep

Logic runs in fixed steps from an accumulator, here at 5, 10, 20 or 60 steps per second so the effect is
visible. The red square is drawn at the latest logic position and jumps from step to step. The blue one is
drawn between the last two logic states with `alpha = leftover time / step` and stays smooth. Key S adds a
stall of half a second: the frame time is clamped to 0.25 s, so the loop does not run hundreds of catch-up
steps (the spiral of death).

See also [delta-time](../delta-time) for the variable step and [update-loop](../update-loop) for the full loop.
