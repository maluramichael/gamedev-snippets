# easing

Easing functions and tweens, drawn. A gallery shows eleven families (linear, sine, quad, cubic, quart, quint, expo, circ,
back, elastic, bounce) as small cards: the curve f(t), a dot for "now" and a square that runs between two points driven
by the same f. In / out / inOut switch the variant for all cards, a click picks one for the detail view with a big plot,
an object with position, scale and rotation and a ghost trail that makes the speed visible. Tab shows `Tween<T>` at work:
three tweens in a row (move, grow, tint), where each one hands the time it did not use to the next. A fixed 20 or 5 fps
step shows that the chain still ends at the same time. The functions and the tween are in `logic.hpp` and covered by `test.cpp`.

Write-up: https://malura.de/en/blog/easing-and-tweening
