# pixel-water

A falling sand cellular automaton with water, sand and stone, drawn as one small texture (240 x 104 cells, every cell
one pixel, scaled 4x with nearest filtering). Hold the mouse to paint a material, the chips change material and brush
size, and a tap at the top keeps pouring water. Each step moves every particle at most once: rows are visited from the
bottom up so a falling cell is not picked up again, the sideways direction alternates, and water slides up to a few
cells per step so a pool levels out. Sand sinks through water. The rules are in `logic.hpp` and covered by `test.cpp`,
including a test that no particle is created or lost over 1000 steps.
