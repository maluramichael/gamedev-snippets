# input-buffer

Coyote time and input buffer for a platformer. A tester runs over stepping stones and presses jump a little too late
at a ledge and a little too early before a landing. Coyote time keeps a jump possible for a moment after the ground is
gone, the input buffer keeps an early press alive until the player can use it. Both windows (0 to 200 ms) are set with
chips, a timeline shows the last three seconds: when jumping was allowed, every press with its waiting window and the
jumps that fired. "you play" hands the controls over (space or click), "reset on jump" switches off the reset of the coyote
timer, which makes a second tap in the air a double jump. The simulation runs in fixed ticks of 1/120 s, so 100 ms is
exactly 12 ticks at any frame rate. The logic (windows, controller, level, tester) is in `logic.hpp` and covered by `test.cpp`.
