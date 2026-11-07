# work-priorities

How RimWorld decides what a colonist does next, as a small simulation. Three colonists, three kinds of work (walls to build,
stones to carry to the stock, meals to cook) and a grid of priorities from 1 to 4 or empty. Click a cell to change it. The rule
from the game: an empty cell never, the smallest number first, equal numbers go to the column further left, and a level is
finished before the next starts. Inside one work type the colonist takes the nearest task, that part is this demo's own choice.
Priority 1 on hauling means every stone is carried across the map before a single wall goes up. The second mode, Score, adds
a walking-distance term (distance plus 150 per priority step) so the numbers become soft. Both modes are simulated to the end
for the current grid, so walked distance and time can be compared at once. The logic is in `logic.hpp` and covered by `test.cpp`.
