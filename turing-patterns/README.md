# turing-patterns

Reaction-diffusion with the Gray-Scott model on a 160x105 grid that wraps around its edges. Two chemicals, A spreads
fast and B slowly, B turns A into more B, A is fed in at a rate `feed` and B is removed at a rate `kill`. Two numbers
decide between spots, stripes, coral and cells that divide. Pick one of the four presets, or drag in the feed/kill map
and watch the running pattern flip. The orange line in the map is `k = sqrt(f) / 2 - f`: beyond it the reaction has no
uniform mixture of A and B any more, the interesting patterns live around it. Left mouse button paints B, right button
erases, the arrow keys nudge feed and kill. The two orange switches show typical mistakes: updating the grid in place
(the scan order leaks into the pattern, try mitosis, which starts mirror symmetric) and a time step above the diffusion
limit of 1.25 (a checkerboard grows until the clamp stops it). Presets 3 and 4 use the values from
[Karl Sims' tutorial](https://www.karlsims.com/rd.html), the other two were found by trying. The logic (`logic.hpp`)
has no SDL dependency and is covered by `test.cpp`.
