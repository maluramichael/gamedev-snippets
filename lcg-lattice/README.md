# lcg-lattice

What a bad random number generator looks like. Left, every pair of consecutive numbers (x_n, x_n+1) is one dot: a linear
congruential generator cannot hide that the next number is a linear function of the previous one, so the dots fall onto
parallel lines (the tiny LCG as a regular grid, the 32 bit LCG and Park-Miller as lines once the zoom narrows the strip
of x to a few line spacings). Right, one bit of every number is a row of coins, and the bit row below it marks which bits
repeat within 1024 draws: bit k of a power-of-two LCG that returns its whole state repeats every 2^(k+1) draws, so `x % 2` strictly alternates and
`x % 6` alternates odd and even. The C library `rand()` of the C standard and of Microsoft hides this by returning only
bits 16 and up. Eight generators to compare (tiny LCG, a typical 32 bit LCG, ANSI `rand()`, MSVC `rand()`, Park-Miller,
xorshift32, PCG32 and the 256 byte table of Wolfenstein 3D). The generators and the statistics are in `logic.hpp`,
checked by `test.cpp` against published sequences and hand-computed cases.
