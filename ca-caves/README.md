# ca-caves

Caves from a cellular automaton, pass by pass. The map starts as random walls, then the 4-5 rule runs again and again: a wall
stays if at least 4 of its 8 neighbors are walls, a floor cell becomes a wall if at least 5 are. The first picture is a finished
cave, "R Reroll" starts again from noise and plays the passes (Space steps one pass, A plays on its own). The slider moves the
wall density over the same random numbers. A click on a cave starts a flood fill: what it reaches turns teal, the caves it cannot
reach are cut off from the rest and turn orange. "Keep largest" fills in everything but the biggest cave. Chip 2 shows the classic
mistake of writing the new value into the map that is still being read (in place) instead of into a copy (double buffer).
The rule, the flood fill and the pipeline are in `logic.hpp` and covered by `test.cpp` (`ca-caves-logic --survey` prints the
numbers over 200 seeds that the write-up quotes).
