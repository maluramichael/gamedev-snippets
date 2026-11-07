# rule-30-random

Rule 30 as a source of random bits. Left, the textbook picture: one black cell, every following row follows from the one
above with the rule `left xor (center or right)`. The left edge is regular, the middle looks like noise. Right, the bits of
one column as a 256 x 256 image (65,536 bits) with the counts a quick check looks at: ones, longest run of equal bits and
the shortest repeat period up to 64. The source is the center column, a diagonal eight cells from the left edge (regular, it
repeats every 4 bits) or, for comparison, one bit of a 32 bit LCG: bit 0 is perfectly balanced and still alternates, bit 31
passes all three counts. Two engines compute the rows, one cell at a time or 64 cells per `uint64_t` with a shift and the
carry between words, and the demo shows how long each needs to fill the image. The automaton, the statistics and the
formatting are in `logic.hpp`, covered by `test.cpp` (hand-computed rows, 100,000 bits against numbers from an independent
Python run, both engines against each other).
