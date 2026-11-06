# idle-economy

The money side of an idle game in four tiers (stand, shop, factory, bank). Every purchase costs `base * g^n` with `g = 1.15`
by default, every building adds a fixed income, and an auto buyer always saves for the tier with the best ratio of waiting
time plus price to income gained. Time can run at x1 up to x86400, so a whole day passes in a second. The plots use a
logarithmic time axis, and there the number of buildings owned is a straight line: with exponential prices every tenfold of
time buys about the same number of buildings. The chip "2 Linear" swaps in a price that grows only linearly (the naive
version), the slider changes `g`, and "P Prestige" resets the run for points that raise all income. Earlier runs stay in the
plot as faded curves, so the shift caused by prestige is visible. Clicking a building row buys 1, 10 or the maximum.

The simulation is event driven: income is constant between purchases, so the wait for the next one is a division and the
result does not depend on the frame rate. Per call the number of purchases is capped, so at very high speed with linear
prices the clock can run slower than the chip says. Numbers are formatted by hand (thousands separators, suffixes, exponent),
without locale. The logic is in `logic.hpp` and covered by `test.cpp` (costs, overflow, bulk prices, hand-computed runs,
slicing invariance, number formatting).
