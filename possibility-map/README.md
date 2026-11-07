# possibility-map

An unseen guard is not simulated. A cloud of question marks grows over every cell where he could be, one cell per tick, and
nothing is decided until the player looks: when the view cone (or, on a noisy floor, the player's ears) touches a possible
cell, the AI has to choose right then. Either the guard appears there and nowhere else is possible any more, or the seen cells
are crossed off. The odds are fair (seen states over all possible states), so two partial looks add up to the same chance as
one full look. WASD walks, the mouse turns the cone, the slider sets how fast the cloud grows. "Naive" turns every observed
question mark into a guard, the classic bug. "Medipack" forks the cloud into "has taken it" and "has not" until someone looks
at the pack, "Noisy floor" makes the door of the guard's room count as observed while the player is within earshot. After
John Manslow, "Possibility Maps for Opportunistic AI and Believable Worlds", Game AI Pro 2, chapter 7. The logic is in
`logic.hpp` and covered by `test.cpp`.
