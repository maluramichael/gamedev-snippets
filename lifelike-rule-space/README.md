# lifelike-rule-space

Every life-like cellular automaton is two sets of digits from 0 to 8: Birth (a dead cell with that many live neighbors
becomes alive) and Survive (a live cell with that many stays alive). `B3/S23` is Conway's Life, and the same notation
describes 262,144 rules in total (nine birth bits, nine survive bits). The chips B0 to B8 and S0 to S8 edit the rule while
it runs on a 156x84 torus, the presets load Life, HighLife, Seeds, Day & Night and Diamoeba with a fresh random soup, "Random
rule" draws one of the 262,144, the R-pentomino starts every rule from the same five cells and the replicator (a ring of 12 cells) copies itself every 12 generations in HighLife. The panel names what the run
does so far (dies out, still, periodic, growing, active) from the exact grid history and a density graph, and points out the
two things the digits alone already prove: with B1 (and no B0) a pattern spreads one cell per side and generation, and
without any of B0 to B3 nothing can be born outside a pattern's bounding box. You can also draw on the grid. The rule parser,
the torus stepper and the run classifier are in `logic.hpp` and covered by `test.cpp`.
