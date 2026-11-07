# jansen-linkage

Theo Jansen's Strandbeest leg as forward kinematics. One crank turns, nothing else is driven: every other joint is the
intersection of two circles whose radii are rod lengths, so the foot path follows from eleven numbers and a crank angle.
Pick a rod with the chips and drag the slider to change it by up to 20 percent, the gray path is the original one. Some
rods tilt the path, others make the crank rock back and forth because two circles stop meeting. Two circles meet in two
points, so five joints can be assembled in 32 ways. "Sides" switches between the intersections remembered from the
start pose (Jansen's leg) and the first one at every joint, which also assembles but into a different leg. The rod
lengths are the table on Wikipedia, the joints each rod connects come from its drawing "Strandbeest Leg Proportions", and
the distances in that drawing reproduce the table. The logic is in `logic.hpp` and covered by `test.cpp`.
