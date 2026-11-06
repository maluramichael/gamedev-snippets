# damage-dice

Thousands of dice rolls as a live histogram. NdS + bonus (1 to 10 dice with 4, 6, 8, 10, 12 or 20 sides) is rolled 400 times
per frame up to a million; the teal bars are what was rolled, the black line is the exact distribution, computed by
convolving the die with itself. A target ("need 12+") tints the area that reaches it and shows its chance. Three boxes
compare exact, rolled and the wrong way: one flat roll over the same range (same mean, much wider spread) and `% sides`
on a toy 8 bit source (the low faces of a d20 come up 13 times in 256 instead of 12). The third box shows what one more
point of bonus is worth at the current target: the height of the bar just below it, 5 points everywhere on a d20, 12.5 in
the middle of 3d6 and 0.5 at 19. Table mode replaces the dice by a hand made weight table (click a bar: weight +1, right
click: -1) that is picked from with running sums and a binary search. Click or drag in the plot to set the target. The odds,
the rejection method, the sampling and the plot geometry are in `logic.hpp` and covered by `test.cpp`, including a
reference value of the PCG32 generator and a count of every byte through the biased d20.
