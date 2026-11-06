# raytracing

A small CPU ray tracer: one ray per pixel into a scene of three balls (one mirror, two matt) on a checkerboard floor, written to a
pixel buffer and drawn scaled up. The mouse moves a point light. Shadows (one extra ray per lit point) and reflections (recursive,
at most 3 bounces) can be switched off, and the resolution changed between 80x45, 160x90 and 320x180, so the stats show what each costs.
The trace is spread over frames with an 8 ms budget, so slow machines see a sweep instead of a stall. The math is in `logic.hpp` and is
tested by `test.cpp` (sphere and plane hits, reflection, shadow acne, bounce limit).
