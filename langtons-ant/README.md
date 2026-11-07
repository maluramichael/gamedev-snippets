# langtons-ant

Langton's ant on a grid that wraps around. On a white cell the ant turns right, on a black one left, flips the cell and steps
forward. The classic ant draws small symmetric patterns for a few hundred steps, then behaves chaotically, and after step 9,977
it builds a "highway": a pattern of 104 steps it repeats forever while moving two cells diagonally per repetition. The demo
recognizes it by measuring (the turns repeat for three periods), shows the phase and a timeline of phases, and starts 6,000 steps in.
On a small torus the highway runs into its own earlier mess and breaks, so the grid size is a chip (50, 100, 200). The other
rules are multi-color ants: every letter of a rule like RLLR is the turn for one color. Speed goes from 1 to 100,000 steps per
second. The ant, the symmetry check and the highway detector are in `logic.hpp` and covered by `test.cpp`, whose reference numbers
come from a separate Python implementation.
