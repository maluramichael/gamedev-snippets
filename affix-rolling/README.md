# affix-rolling

A mini item generator in the style of loot-based action RPGs, with the rolling spelled out. Pick a base item and an item
level, then use crafting orbs (Transmute, Augment, Regal, Alchemy, Exalt, Chaos). The middle panel shows the pool for the
next mod: every group that fits the item, the tiers the item level has opened (dots) and the chance of each tier (bar).
The right panel rolls 2000 fresh items with one Alchemy orb and counts the tiers. Item level only opens tiers, the
weights keep the best ones rare, and a group is on the item at most once. The two orange switches turn on the classic
mistakes: every tier as one ticket (flat weights) and a group that can be drawn again. The pool is the demo's own example
data, it is not a table of any game. The logic is in `logic.hpp` and covered by `test.cpp`.
