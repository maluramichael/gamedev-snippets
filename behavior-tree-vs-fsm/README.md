# behavior-tree-vs-fsm

Two guards walk the same corridor and decide the same things (patrol, chase, attack, search), one with a finite state
machine (left: states and edges), one with a behavior tree (right: every node lit green, red or yellow by what it returned
this tick). The mouse is the player, a click makes noise. Three chips add requirements: a noise to check, low health that
sends the guard to a medkit, a radio call that interrupts everything. The state machine needs an edge from every existing
state for the last two, the tree gets one branch at the top, and the counters show both costs. The selector chip makes the
tree passive: a running branch keeps control until it is done, so the guard ignores you while it searches. The history chip
gives the state machine a marker that remembers where the radio call came from. The tree needs no marker, it looks at the
top again after the call. The tree nodes follow the Behavior Tree Starter Kit (sequence, selector, active selector), the
logic is in `logic.hpp` and covered by `test.cpp`.
