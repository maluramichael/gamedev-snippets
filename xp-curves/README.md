# xp-curves

How much XP does the next level cost? Five curve families (linear `a*L`, polynomial `a*L^b`, exponential `a*b^L`, the RuneScape
formula and a hand-picked table) are plotted as "XP to the next level" and "total XP", on a linear or logarithmic y axis, for
the first 30, 60 or 99 levels. Two sliders change a and b, a third sets the XP per hour, which turns the curve into play time
in the lower plot. Hover the plots to read level, cost, total and hours. A simulated player earns XP in fixed steps and
climbs the curve (x1, x10, x100). The curve math, the level lookup (binary search over the thresholds), the 64 bit
saturation and the number formatting are in `logic.hpp` and covered by `test.cpp`, which also checks the RuneScape
values (level 2 = 83, level 10 = 1,154, level 50 = 101,333, level 99 = 13,034,431).

Write-up: https://malura.de/en/blog/xp-curves
