# fizzle-fade

The screen transition of Wolfenstein 3D: every pixel changes exactly once, in a scrambled order, at constant speed. On the left the
version everybody writes first (pick a random pixel, check a flag, pick again if it was taken), with a counter for the wasted tries and
one flag per pixel. On the right a linear feedback shift register of 13, 15 or 17 bits, cut into x and y, that visits every pixel exactly once
and needs no memory beyond its own state. Both get the same number of tries per second, a chart below shows the covered share over the
tries: the random picker starts ahead and falls behind around the halfway mark of the register's cycle, because the register also steps
through states outside the screen. Chips switch the resolution (100x60, 200x120, 400x240) and the speed, R restarts, P or Space pauses.
The logic is in `logic.hpp` and covered by `test.cpp`: every tap mask has the maximum period, and for a dozen resolutions each pixel
comes up exactly once.
