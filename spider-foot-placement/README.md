# spider-foot-placement

A spider seen from above follows the mouse with six legs of two segments each (law-of-cosines IK, knees bent outward).
The body is driven, the feet are not: every foot stays where it is in the world until the point it should stand on, its
rest point plus the body velocity times a short look-ahead, has moved further than a threshold away. Then it swings
there in an arc. The gait is only a rule about who may start a swing: alternating tripod (two groups of three, one
group in the air at a time), crawl (one leg at a time) or free (every leg that wants to). The polygon spanned by the
planted feet is drawn against the center of the body, and the panel counts feet on the ground, time without support and
the duty factor. Chip L turns the look-ahead off: the feet then land on the rest point and the stride lies behind it,
the legs stretch to their limit and the body has to slow down. A planted foot never moves by itself, the body is the one
that gives way when a leg would reach beyond 97 percent of its length. The logic is in `logic.hpp` and covered by `test.cpp`.
