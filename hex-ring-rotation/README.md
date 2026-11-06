# hex-ring-rotation

Rings, spirals, turning and mirroring on a hex grid (axial coordinates, flat-top hexes). The Ring and Spiral modes walk a
ring or a spiral of rings cell by cell around the origin: the six walking directions, the cell count and its formula
`1 + 3N(N+1)` are shown next to the board (radius with `-` and `+`, `R` replays). The Rotate mode turns a small shape
around a pivot you click, in 60 degree steps (`Q` left, `E` right) and mirrors it (`4`, `5`, `6`), and prints the swap of
the cube coordinates that does it. `N` switches to the naive way, q and r read as x and y, sine and cosine, round, which
tears the shape apart; `0` resets. The logic is in `logic.hpp` and covered by `test.cpp`.

The idea follows the hex guide by Red Blob Games: https://www.redblobgames.com/grids/hexagons/ (own words, own code).
