# verlet-cloth

A curtain, two ropes and a hammock made of points and distance links, simulated with Verlet integration: a point stores
only its position and the one from the step before, so the velocity is their difference. Every step the links are pushed
back to their rest length several times (the Iterations slider: one pass is rubber, thirty are cloth). Grab points with the
mouse, pin them, cut threads by swiping across them, let threads tear when they stretch beyond 2.5 times their length, and
change the wind. The "Stored velocity" chip switches to the naive variant where the velocity has its own variable that the
corrections never touch: hanging points pile up speed (the probe shows it) and shoot away when you cut the thread. The
simulation is in `logic.hpp` and covered by `test.cpp`. There is no self collision and no shear threads, which is
mentioned in the post.
