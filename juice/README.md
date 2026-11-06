# juice

A hero hits a dummy and seven small reactions can be switched on one by one: hit stop, camera shake (trauma model), a white
flash, squash and knockback, sparks, an impact ring and a damage number. With all of them off the hit is a number going
down, with all of them on the same 0.1 s carries a lot more weight. Click or Space hits, right click or Enter hits hard.
The timeline under the stage shows what the hit stop really pauses: game time (teal turns orange), while the camera, the
flash and your clicks keep running on real time. Two chips show the usual mistakes: "Stop eats input" drops every click
that arrives during the freeze, "Overdo shake" shakes the screen on every swing so the real hits no longer stand out.
"Reduce motion" turns off shake, flash, squash and sparks, "Slow-mo" runs everything at a quarter speed. At most one flash
per hit and a hit takes at least 0.42 s, so nothing blinks faster than 3 times a second. The logic is in `logic.hpp` and
covered by `test.cpp`.

Talk the idea comes from: [Juice It or Lose It](https://www.gdcvault.com/play/1016487/Juice-It-or-Lose) (GDC Europe 2012).
