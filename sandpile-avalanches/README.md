# sandpile-avalanches

The abelian sandpile on a 129 x 129 grid. Grains land on cells, a cell with 4 or more grains topples and gives one grain to each of
its four neighbors for every 4 it holds, grains pushed over the edge are gone (a corner cell loses 2 of its 4 per toppling, an
edge cell 1). The first picture is 20000 grains poured into the middle. The Rows, Random and Reverse chips decide in which order the
unstable cells of a pass are visited, and with "Animate" on you can watch the intermediate pictures differ. The end picture does not:
the panel keeps a fingerprint (FNV-1a over all cells) and the topple count of the last pour per order, and they are identical.
"Fill 2" with "Run" drops random grains on a grid that is already close to full. Most grains change nothing, a few start an
avalanche over most of the grid, the histogram counts the avalanche sizes in octaves on a log scale. A click drops a grain,
holding the button rains. The model, the fingerprint and the histogram are in `logic.hpp` and covered by `test.cpp`
(`sandpile-avalanches-logic --survey` prints the numbers the write-up quotes).
