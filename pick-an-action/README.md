# pick-an-action

Utility AI, second half: the scores are fixed, what changes is how the agent picks from them. Four need sliders feed six
actions (Eat, Snack, Sleep, Play, Chat, Watch TV). Four selection rules read the same scores: Max (always the best, ties split),
weighted random (chance = score / sum), a band (weighted random among scores within 30 percent of the best) and dual utility
(only the highest rank survives, weights decide inside it; a need at 0.8 or more lifts its action to rank 2, Watch TV is
a rank 0 filler). Every change rolls 1000 picks. The tick on each bar is the exact chance, the bar is what was rolled,
orange marks actions scoring under half of the best, and the stupidity rate is the share of picks that were one of them.
Space adds 1000 more. The logic is in `logic.hpp` and covered by `test.cpp`.
