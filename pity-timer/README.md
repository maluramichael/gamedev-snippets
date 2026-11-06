# pity-timer

Ten thousand players roll for the same rare drop until it drops, and the histogram shows how long each had to wait. With
plain random a 1 % rate leaves about 37 % of the players empty-handed after 100 tries, because every try starts from
scratch. Two fixes let the chance depend on the misses so far: a pseudo-random distribution (the chance on try n is
`C * n`, with `C` chosen so the long-run rate stays exactly 1 %) and soft plus hard pity (base rate, a linear ramp from
three quarters of the guarantee, 100 % at the guarantee). Switch between the three, change the drop rate and the
guarantee, and roll again. The bars are the dice, the black line is the exact distribution computed in `logic.hpp`, the
thin orange line is plain random for comparison. The ramp start for pity is a choice of this demo, not the setting of any
game. The logic is covered by `test.cpp`.
