# elementary-rules

All 256 elementary cellular automata on a ring of cells. The eight cards on top are the rule: each shows a neighborhood
(left, center, right) and what the middle cell becomes, and a click on that lower cell flips the bit. The rule number from 0
to 255 is those eight bits read as a binary number. The picture is the rule at work, one row per step, time running downward.
Chips load rules 30, 90, 110 and 184, Mirror and Complement switch to the related rule, and Variants draws the rule and its
three relatives side by side: mirroring a rule gives the left-right flipped picture, complementing it gives the inverted one.
That is why the 256 rules are only 88 different worlds, and why rule 110 comes back as 124, 137 and 193. Start from a single
cell or a random row. The edges wrap around. The automaton and the symmetries are in `logic.hpp` and covered by `test.cpp`.
