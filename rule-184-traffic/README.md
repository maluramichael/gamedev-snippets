# rule-184-traffic

Traffic on a ring road from one cellular automaton rule. A car drives if the cell ahead is free and waits otherwise, all cars
decide at the same time. Every row of the picture is one time step, the newest at the bottom, teal cars drive and orange cars
wait. Below a density of one half every jam dissolves and all cars drive. Above it the jams stay and drift backwards, against the
driving direction, while the number of cars never changes. The chart on the right plots the speed over the density: the line is
`min(1, (1 - d) / d)`, the small dots were measured by the demo itself at start-up, the big dot is the current road. The slider
changes the density, a click on the picture adds or removes a car, "Queue" puts all cars in one row and shows how a queue
dissolves from its front, one car per step. "In place" shows the classic mistake of updating the road in one sweep over one array:
a car that has just moved is met again and moves again, so on a free road the speed is the mean gap instead of 1. The rule, the
measurement and the mistake are in `logic.hpp` and covered by `test.cpp` (`rule-184-traffic-logic --survey` prints the numbers
the write-up quotes).
