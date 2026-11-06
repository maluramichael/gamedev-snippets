# damage-stacking

Two percentage bonuses that look the same in a tooltip and behave very differently. Six lines of +50 % sit in two pots.
Everything in the Increased pot is added up and applied once (1 + 0.5 + 0.5 + ...), every line in the More pot multiplies
on its own (1.5 * 1.5 * ...). A click moves a line to the other pot, the mouse wheel changes its percentage in steps of
5 (-50 to +200), keys 1 to 6 move the lines. The upper right shows the total damage next to the two naive readings ("add
every percentage", "multiply every percentage"), the lower right shows what each line is really worth next to what its label
says: a More line keeps its printed value, an Increased line loses value with every other line in its pool. Nothing goes
below zero, so a pool of -150 % or two factors of -150 % cannot turn the damage negative or positive again. The math is in
`logic.hpp` and covered by `test.cpp`. All numbers (base damage 100, +50 %) are example values, not those of a real game.

The idea is known from Path of Exile, see the forum thread "Increased, more, addictive, multiplicative--what is the
difference?" (https://www.pathofexile.com/forum/view-thread/261646).
