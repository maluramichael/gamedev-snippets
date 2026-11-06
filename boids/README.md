# boids

Flocking after Craig Reynolds: every bird follows three rules that only look at the birds within 50 px (separation,
alignment, cohesion), nobody leads and nobody knows the flock. Three sliders weigh the rules, the mouse is a predator
the birds flee from. The world wraps around at the edges. The second half is the neighbor search: asking every bird about
every other one is n^2 distance tests per step, a grid with cells as wide as the perception only asks the nine cells
around a bird. Switch between Grid search and Brute force and watch tests per step, step time and frames per second
(4000 birds shows the difference best), "Show grid" draws the cells one bird looks at. The rules, the grid and the
wrap-around distances are in `logic.hpp` and covered by `test.cpp`.

Idea and rules: https://www.red3d.com/cwr/boids/ (the code is my own).
